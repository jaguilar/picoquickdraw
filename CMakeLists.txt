cmake_minimum_required(VERSION 3.25)

add_subdirectory(lib/picotemplate)

project(picoquickdraw VERSION 0.1.0 LANGUAGES C CXX ASM)    

pico_sdk_init()

add_subdirectory(src)