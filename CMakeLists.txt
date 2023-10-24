cmake_minimum_required(VERSION 3.16)

project(MapNLQ LANGUAGES CXX)

set(sources ${CMAKE_CURRENT_SOURCE_DIR}/src/MapNLQ.cpp)

if (WIN32)
    set(sources ${sources} ${CMAKE_CURRENT_SOURCE_DIR}/src/MapNLQ.rc)
endif()

add_library(MapNLQ SHARED ${sources})

target_include_directories(MapNLQ PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

if (UNIX)
    target_include_directories(MapNLQ PRIVATE
    /usr/local/include/avisynth
    /usr/local/include
    )
else if (MINGW)
    target_include_directories(MapNLQ PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/dovi_tool/dolby_vision/target/x86_64-pc-windows-gnu/release/include)
else()
    target_include_directories(MapNLQ PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/dovi_tool/dolby_vision/target/x86_64-pc-windows-msvc/release/include)
endif()

if (NOT CMAKE_GENERATOR MATCHES "Visual Studio")
    if (NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
    endif()

    string(TOLOWER ${CMAKE_BUILD_TYPE} build_type)
    if (build_type STREQUAL Debug)
        target_compile_definitions(MapNLQ PRIVATE DEBUG_BUILD)
    else (build_type STREQUAL Release)
        target_compile_definitions(MapNLQ PRIVATE RELEASE_BUILD)
    endif()

    message(STATUS "Build type - ${CMAKE_BUILD_TYPE}")
endif()

target_compile_features(MapNLQ PRIVATE cxx_std_17)

find_library(libdovi dovi PATHS
    ${CMAKE_CURRENT_SOURCE_DIR}/dovi_tool/dolby_vision/target/x86_64-pc-windows-msvc/release
    ${CMAKE_CURRENT_SOURCE_DIR}/dovi_tool/dolby_vision/target/x86_64-pc-windows-gnu/release
)
message(STATUS "libdovi: ${libdovi}")

target_link_libraries(MapNLQ PRIVATE
    avisynth
    ${libdovi}
)

if (WIN32)
    target_link_libraries(MapNLQ PRIVATE
        ws2_32
        bcrypt
        userenv
        shlwapi
        ntdll
    )
endif()

if (MINGW)
    set_target_properties(MapNLQ PROPERTIES PREFIX "")

    target_link_libraries(MapNLQ PRIVATE -static-libgcc -static-libstdc++ -s -static)
endif()

if (UNIX)
    find_package (Git)

    if (GIT_FOUND)
        execute_process (COMMAND ${GIT_EXECUTABLE} describe --tags --abbrev=0
            OUTPUT_VARIABLE ver
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )
        set_target_properties(MapNLQ PROPERTIES OUTPUT_NAME "MapNLQ.${ver}")
    else()
        message (STATUS "GIT not found")
    endif()

    include(GNUInstallDirs)

    INSTALL(TARGETS MapNLQ LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}/avisynth")

    # uninstall target
    if(NOT TARGET uninstall)
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
        IMMEDIATE @ONLY)

    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
    endif()
endif()
