cmake_minimum_required(VERSION 3.13)

include(FetchContent)
include("cmake/HunterGate.cmake")

HunterGate(
    URL "https://github.com/cpp-pm/hunter/archive/ead00a62a9a05d36f3d376889360b379d9046ba1.tar.gz"
    SHA1 "7456937b43b9071498ba521fda9daafd931baa4a"
)

project(SchwarzwaldRoot)

set(LASZIP_INCLUDE_DIRS "" CACHE STRING "Include directory for LASzip")
set(LASZIP_LIBRARY "" CACHE STRING "Path to LASzip library file")

set(TL_EXPECTED_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/lib/tl_expected")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(!UNIX)
add_compile_definitions(_SILENCE_CXX17_ITERATOR_BASE_CLASS_DEPRECATION_WARNING)
endif()

if(UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wno-unknown-pragmas -std=c++17 -lm")
    #SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS}" )
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/build/Debug)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${CMAKE_SOURCE_DIR}/build/RelWithDebInfo)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/build/Release)

add_subdirectory(schwarzwald/util)
add_subdirectory(schwarzwald/core)
add_subdirectory(schwarzwald/executable)
add_subdirectory(schwarzwald/test)
add_subdirectory(schwarzwald/benchmark)

if (MSVC_VERSION GREATER_EQUAL "1900") 
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("/std:c++latest" _cpp_latest_flag_supported)
    if (_cpp_latest_flag_supported)
        add_compile_options("/std:c++latest")
    endif()
endif()