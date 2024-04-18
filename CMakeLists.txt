cmake_minimum_required(VERSION 3.25)
include(FetchContent)



project(picoquickdraw VERSION 0.1.0 LANGUAGES C CXX ASM)    

pico_sdk_init()

add_subdirectory(src)



FetchContent_Declare(
  picobase
  GIT_REPOSITORY https://github.com/jaguilar/picobase.git
  GIT_TAG        main
)
FetchContent_MakeAvailable(picobase)
include(${picobase_SOURCE_DIR}/basic_imports.cmake)