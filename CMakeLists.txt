cmake_minimum_required(VERSION 3.21)

include(FetchContent)

FetchContent_Declare(
  picobase
  GIT_REPOSITORY https://github.com/jaguilar/picobase.git
  GIT_TAG        main
  EXCLUDE_FROM_ALL
)

FetchContent_MakeAvailable(picobase)
include(${picobase_SOURCE_DIR}/basic_imports.cmake)

FetchContent_Declare(
  jagspico
  GIT_REPOSITORY https://github.com/jaguilar/jagspico.git
  GIT_TAG        main
  EXCLUDE_FROM_ALL
)

project(picoquickdraw VERSION 0.1.0 LANGUAGES C CXX ASM)    

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(PICO_BOARD "pico_w")

pico_sdk_init()

FetchContent_MakeAvailable(jagspico)

add_subdirectory(src)



