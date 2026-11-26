cmake_minimum_required(VERSION 3.25)
project(lumen-tracer VERSION 0.1.0 DESCRIPTION "A physically based path tracer written from scratch" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(LUMEN_WERROR "Treat warnings as errors" OFF)
option(LUMEN_NATIVE "Tune for the build machine (-march=native); binaries are not portable" OFF)
option(LUMEN_COVERAGE "Instrument for llvm-cov source coverage" OFF)
set(LUMEN_SANITIZE "" CACHE STRING "Comma separated sanitizers, e.g. address,undefined or thread")

find_package(Threads REQUIRED)

add_library(lumen_options INTERFACE)
target_compile_options(lumen_options INTERFACE -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)
if(LUMEN_WERROR)
  target_compile_options(lumen_options INTERFACE -Werror)
endif()
if(LUMEN_NATIVE)
  target_compile_options(lumen_options INTERFACE -march=native)
endif()
if(LUMEN_COVERAGE)
  target_compile_options(lumen_options INTERFACE -fprofile-instr-generate -fcoverage-mapping)
  target_link_options(lumen_options INTERFACE -fprofile-instr-generate)
endif()
if(LUMEN_SANITIZE)
  target_compile_options(lumen_options INTERFACE -fsanitize=${LUMEN_SANITIZE} -fno-omit-frame-pointer
                         -fno-sanitize-recover=undefined)
  target_link_options(lumen_options INTERFACE -fsanitize=${LUMEN_SANITIZE})
endif()

file(GLOB LUMEN_SOURCES CONFIGURE_DEPENDS src/lumen/*.cpp)
add_library(lumen_core STATIC ${LUMEN_SOURCES})
target_include_directories(lumen_core PUBLIC src)
target_link_libraries(lumen_core PUBLIC Threads::Threads PRIVATE lumen_options)

add_executable(lumen src/main.cpp)
target_link_libraries(lumen PRIVATE lumen_core lumen_options)

add_executable(lumen-assets tools/gen_assets.cpp)
target_link_libraries(lumen-assets PRIVATE lumen_core lumen_options)

enable_testing()
add_subdirectory(tests)
