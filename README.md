## Description

Polynomial luma mapping, MMR chroma mapping on base layer. NLQ 12 bits restoration from enhancement layer.

This is [a port of the VapourSynth plugin vs-nlq](https://github.com/quietvoid/vs-nlq).

### Requirements:

- AviSynth+ r3688 (can be downloaded from [here](https://gitlab.com/uvz/AviSynthPlus-Builds) or [here](https://github.com/AviSynth/AviSynthPlus/releases)) or later

- Microsoft VisualC++ Redistributable Package 2022 (can be downloaded from [here](https://github.com/abbodi1406/vcredist/releases))

#### Usage:

```
MapNLQ(clip BL, clip EL, string "rpu")
```

#### Parameters:

- BL<br>
    Base layer.<br>
    It must be Profile 7 (BL+EL+RPU).<br>
    It must be in YUV(A) 16-bit planar format.<br>
    It must have frame property `DolbyVisionRPU`.

- EL<br>
    Enhancement layer<br>
    It must be Profile 7 (EL+RPU).<br>
    It must be in YUV 10-bit planar format.<br>
    It must have the same subsampling as `BL`.<br>
    It must have 0.25 dimension of the `BL` dimension.<br>
    It must have the same number of frames as `BL`.<br>
    If `rpu` isn't specified, this must have frame property `DolbyVisionRPU`.

- rpu<br>
    Path to the RPU binary.<br>
    Default: Not specified.

The output is 12-bit.

#### Script example

```
BL_clip
z_ConvertFormat(bit_depth=16)
Prefetch(2)
libplacebo_Tonemap(src_csp=3, dst_csp=1)
Prefetch(1)
z_ConvertFormat(pixel_type="yuv420p16", chromaloc_op="top_left=>top_left", resample_filter="spline36")
bl=Prefetch(2)

el=EL_clip

MapNLQ(bl, el)
#MapNLQ(bl, el, rpu)
Prefetch(2)
```

### Building:

```
Requirements:
- Git
- C++17 compiler
- CMake >= 3.16
```

```
git clone --recurse-submodules https://github.com/Asd-g/AviSynthPlus-MapNLQ

cd AviSynthPlus-MapNLQ/dovi_tool/dolby_vision

cargo install cargo-c
cargo cbuild --release

cd ../../

cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release

cmake --build build
```
