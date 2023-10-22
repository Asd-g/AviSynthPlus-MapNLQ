#include <algorithm>
#include <array>
#include <string>

#include "avisynth_c.h"
#include "libdovi/rpu_parser.h"

struct MapNLQ
{
    AVS_Clip* el;
    const DoviRpuOpaqueList* rpu;
    std::string error_msg;
};

static AVS_VideoFrame* AVSC_CC MapNLQ_get_frame(AVS_FilterInfo* fi, int n)
{
    MapNLQ* d{ reinterpret_cast<MapNLQ*>(fi->user_data) };

    AVS_VideoFrame* bl_frame{ avs_get_frame(fi->child, n) };
    if (!bl_frame)
        return nullptr;

    AVS_VideoFrame* el_frame{ avs_get_frame(d->el, n) };
    if (!el_frame)
    {
        avs_release_video_frame(bl_frame);
        return nullptr;
    }

    const auto error{ [&](const std::string& msg)
    {
        avs_release_video_frame(bl_frame);
        avs_release_video_frame(el_frame);

        d->error_msg = msg;
        fi->error = d->error_msg.c_str();

        return nullptr;
    }
    };

    DoviRpuOpaque* rpu{ [&]() -> DoviRpuOpaque*
    {
        if (d->rpu)
        {
            if (n >= d->rpu->len)
                return nullptr;
            else
                return d->rpu->list[n];
        }
        else
        {
            const AVS_Map* props{ avs_get_frame_props_ro(fi->env, el_frame) };
            if (avs_prop_num_elements(fi->env, props, "DolbyVisionRPU") > -1)
            {
                const uint8_t* rpu_prop{ reinterpret_cast<const uint8_t*>(avs_prop_get_data(fi->env, props, "DolbyVisionRPU", 0, nullptr)) };
                const size_t rpu_size{ static_cast<size_t>(avs_prop_get_data_size(fi->env, props, "DolbyVisionRPU", 0, nullptr)) };
                if (rpu_prop && rpu_size)
                    return dovi_parse_unspec62_nalu(rpu_prop, rpu_size);
                else
                    return nullptr;
            }
            else
                return nullptr;
        }
    }()
    };
    if (!rpu)
        return error("MapNLQ: cannot parse RPU.");

    const DoviRpuDataMapping* mapping{ dovi_rpu_get_data_mapping(rpu) };
    const int num_pivots{ mapping->nlq_num_pivots_minus2 + 1 };
    if (num_pivots != 1)
    {
        if (!d->rpu)
            dovi_rpu_free(rpu);
        dovi_rpu_free_data_mapping(mapping);
        return error("MapNLQ: wrong pivots number.");
    }

    const DoviRpuDataNlq* rpu_data_nlq{ mapping->nlq };
    const DoviRpuDataHeader* header{ dovi_rpu_get_header(rpu) };
    if (!header)
    {
        std::string err{ dovi_rpu_get_error(rpu) };
        if (!d->rpu)
            dovi_rpu_free(rpu);
        dovi_rpu_free_data_mapping(mapping);
        return error(err);
    }
    if (header->guessed_profile != 7)
    {
        if (!d->rpu)
            dovi_rpu_free(rpu);
        dovi_rpu_free_data_mapping(mapping);
        dovi_rpu_free_header(header);
        return error("MapNLQ: the RPU profile isn't 7.");
    }

    const uint64_t out_bit_depth{ header->vdr_bit_depth_minus8 + 8 };
    const uint64_t el_bit_depth{ header->el_bit_depth_minus8 + 8 };
    const uint64_t coeff_log2_denom{ header->coefficient_log2_denom };
    const bool disable_residual_flag{ header->disable_residual_flag };

    AVS_VideoFrame* dst{ avs_new_video_frame_p(fi->env, &fi->vi, bl_frame) };

    const int64_t maxout = (1 << out_bit_depth) - 1;
    const uint64_t coef{ (!disable_residual_flag) ? (coeff_log2_denom - 5 - el_bit_depth) : 0 };
    const int64_t ou{ 1LL << (15 - out_bit_depth) };
    const uint64_t ou1{ 16 - out_bit_depth };
    constexpr std::array<int, 3> planes{ AVS_PLANAR_Y, AVS_PLANAR_U, AVS_PLANAR_V };

    for (int i{ 0 }; i < 3; ++i)
    {
        const int bl_width{ avs_get_row_size_p(bl_frame, planes[i]) / 2 };
        const int bl_height{ avs_get_height_p(bl_frame, planes[i]) };

        const int bl_pitch{ avs_get_pitch_p(bl_frame, planes[i]) / 2 };
        const int el_pitch{ avs_get_pitch_p(el_frame, planes[i]) / 2 };
        const int dst_pitch{ avs_get_pitch_p(dst, 1 << i) / 2 };

        const uint16_t* blp{ reinterpret_cast<const uint16_t*>(avs_get_read_ptr_p(bl_frame, planes[i])) };
        const uint16_t* elp{ reinterpret_cast<const uint16_t*>(avs_get_read_ptr_p(el_frame, planes[i])) };
        uint16_t* dstp{ reinterpret_cast<uint16_t*>(avs_get_write_ptr_p(dst, planes[i])) };

        int64_t thr{};
        int64_t rr{};

        if (!disable_residual_flag)
        {
            const int64_t thresh{ static_cast<int64_t>((rpu_data_nlq->linear_deadzone_threshold_int[i] << coeff_log2_denom)) + static_cast<int64_t>(rpu_data_nlq->linear_deadzone_threshold[i]) };
            thr = thresh << (10 - el_bit_depth + 1);
            const int64_t fp_in_max{ static_cast<int64_t>((rpu_data_nlq->vdr_in_max_int[i] << coeff_log2_denom)) + static_cast<int64_t>(rpu_data_nlq->vdr_in_max[i]) };
            rr = fp_in_max << (10 - el_bit_depth + 1);
        }

        const int64_t slope{ static_cast<int64_t>((rpu_data_nlq->linear_deadzone_slope_int[i] << coeff_log2_denom)) + static_cast<int64_t>(rpu_data_nlq->linear_deadzone_slope[i]) };

        for (int y{ 0 }; y < bl_height; ++y)
        {
            for (int x{ 0 }; x < bl_width; ++x)
            {
                int64_t tmp{ elp[x / 2] - static_cast<int64_t>(rpu_data_nlq->nlq_offset[i]) };
                int64_t h{ blp[x] };

                if (!disable_residual_flag)
                {
                    const int64_t result{ [&]()
                    {
                        if (!tmp)
                            return 0LL;
                        else
                        {
                            const int sign{ (tmp < 0) ? -1 : 1 };
                            tmp <<= 1;
                            tmp -= sign;
                            tmp <<= 10 - el_bit_depth;

                            int64_t dq{ tmp * slope };
                            dq += (thr * sign);
                            dq = std::clamp(dq, -rr, rr);

                            return dq >> coef;
                        }
                    }()
                    };

                    h += result;
                }

                h += ou;
                h >>= ou1;
                dstp[x] = static_cast<uint16_t>(std::clamp(h, 0LL, maxout));
            }

            blp += bl_pitch;
            dstp += dst_pitch;

            if (y & 1)
                elp += el_pitch;
        }
    }

    if (avs_num_components(&fi->vi) > 3)
        avs_bit_blt(fi->env, avs_get_write_ptr_p(dst, AVS_PLANAR_A), avs_get_pitch_p(dst, AVS_PLANAR_A), avs_get_read_ptr_p(bl_frame, AVS_PLANAR_A), avs_get_pitch_p(bl_frame, AVS_PLANAR_A),
            avs_get_row_size_p(bl_frame, AVS_PLANAR_A), avs_get_height_p(bl_frame, AVS_PLANAR_A));

    avs_release_video_frame(bl_frame);
    avs_release_video_frame(el_frame);
    if (!d->rpu)
        dovi_rpu_free(rpu);
    dovi_rpu_free_data_mapping(mapping);
    dovi_rpu_free_header(header);

    return dst;
}

static void AVSC_CC free_MapNLQ(AVS_FilterInfo* fi)
{
    MapNLQ* d{ reinterpret_cast<MapNLQ*>(fi->user_data) };

    if (d->rpu)
        dovi_rpu_list_free(d->rpu);

    avs_release_clip(d->el);
    delete d;
}

static int AVSC_CC MapNLQ_set_cache_hints(AVS_FilterInfo* fi, int cachehints, int frame_range)
{
    return cachehints == AVS_CACHE_GET_MTMODE ? 2 : 0;
}

AVS_Value AVSC_CC create_MapNLQ(AVS_ScriptEnvironment* env, AVS_Value args, void* param)
{
    enum { Bl, El, Rpu };

    MapNLQ* params{ new MapNLQ() };

    AVS_FilterInfo* fi;
    AVS_Clip* clip{ avs_new_c_filter(env, &fi, avs_array_elt(args, Bl), 1) };

    const auto set_error{ [&](const char* error, AVS_Clip* clip1)
    {
        avs_release_clip(clip);

        if (clip1)
            avs_release_clip(clip1);

        return avs_new_value_error(error);
    }
    };

    if (!avs_check_version(env, 9))
    {
        if (avs_check_version(env, 10))
        {
            if (avs_get_env_property(env, AVS_AEP_INTERFACE_BUGFIX) < 2)
                return set_error("MapNLQ: AviSynth+ version must be r3688 or later.", nullptr);
        }
    }
    else
        return set_error("MapNLQ: AviSynth+ version must be r3688 or later.", nullptr);

    if (avs_is_rgb(&fi->vi) || !avs_is_planar(&fi->vi))
        return set_error("MapNLQ: the clip must be in YUV planar format.", nullptr);
    if (avs_num_components(&fi->vi) < 3)
        return set_error("MapNLQ: the BL clip must have at least 3 planes.", nullptr);
    if (avs_bits_per_component(&fi->vi) != 16)
        return set_error("MapNLQ: the BL clip bit depth must be 16.", nullptr);

    params->el = avs_take_clip(avs_array_elt(args, El), env);

    const AVS_VideoInfo* vi1{ avs_get_video_info(params->el) };

    if (avs_is_rgb(vi1) || !avs_is_planar(vi1))
        return set_error("MapNLQ: the clip must be in YUV planar format.", params->el);
    if (avs_num_components(&fi->vi) != avs_num_components(vi1))
        return set_error("MapNLQ: the clips number of planes doesn't match.", params->el);
    if (avs_bits_per_component(vi1) != 10)
        return set_error("MapNLQ: the EL clip bit depth must be 10.", params->el);
    if (vi1->num_frames != fi->vi.num_frames)
        return set_error("MapNLQ: the clips number of frames doesn't match.", params->el);
    if (avs_get_plane_width_subsampling(&fi->vi, AVS_PLANAR_U) != avs_get_plane_width_subsampling(vi1, AVS_PLANAR_U) ||
        avs_get_plane_height_subsampling(&fi->vi, AVS_PLANAR_U) != avs_get_plane_height_subsampling(vi1, AVS_PLANAR_U))
        return set_error("MapNLQ: the clips subsampling doesn't match.", params->el);
    if ((fi->vi.width != vi1->width << 1) || (fi->vi.height != vi1->height << 1))
        return set_error("MapNLQ: the EL dimension must be 0.25 of the BL dimension.", params->el);

    if (avs_is_420(&fi->vi))
        fi->vi.pixel_type = (avs_is_yuv(&fi->vi)) ? AVS_CS_YUV420P12 : AVS_CS_YUVA420P12;
    else if (avs_is_422(&fi->vi))
        fi->vi.pixel_type = (avs_is_yuv(&fi->vi)) ? AVS_CS_YUV422P12 : AVS_CS_YUVA422P12;
    else
        fi->vi.pixel_type = (avs_is_yuv(&fi->vi)) ? AVS_CS_YUV444P12 : AVS_CS_YUVA444P12;

    if (avs_defined(avs_array_elt(args, Rpu)))
    {
        params->rpu = dovi_parse_rpu_bin_file(avs_as_string(avs_array_elt(args, Rpu)));
        if (params->rpu->error)
        {
            params->error_msg = params->rpu->error;
            dovi_rpu_list_free(params->rpu);

            return set_error(params->error_msg.c_str(), params->el);
        }
    }

    AVS_Value v{ avs_new_value_clip(clip) };

    fi->user_data = reinterpret_cast<void*>(params);
    fi->get_frame = MapNLQ_get_frame;
    fi->set_cache_hints = MapNLQ_set_cache_hints;
    fi->free_filter = free_MapNLQ;

    avs_release_clip(clip);

    return v;
}

const char* AVSC_CC avisynth_c_plugin_init(AVS_ScriptEnvironment* env)
{
    avs_add_function(env, "MapNLQ", "cc[rpu]s", create_MapNLQ, 0);

    return "MapNLQ";
}
