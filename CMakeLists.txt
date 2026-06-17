cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# 绕过 opus 编解码库的编译 warning（新版 GCC 的 stringop-overflow 误报）
add_compile_options(-Wno-error=stringop-overflow)

project(pump_control)
