cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zephyrair)

target_include_directories(app PRIVATE
    src
    src/bmp3_api
)

target_sources(app PRIVATE 
    src/main.c
    src/bmp390_bridge.c
    src/bmp3_api/bmp3.c
    src/bmp390_emul.c
    src/dsp_filter.c
)