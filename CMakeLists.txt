cmake_minimum_required(VERSION 3.6)

include("cmake/HunterGate.cmake")

HunterGate(
    URL "https://github.com/ruslo/hunter/archive/v0.23.103.tar.gz"
    SHA1 "0b8c31bfdf0fd2fa87b4f743aabcd73904afa256"
)

project(Potree)

set(LASZIP_INCLUDE_DIRS "" CACHE STRING "Include directory for LASzip")
set(LASZIP_LIBRARY "" CACHE STRING "Path to LASzip library file")
set(TERMINALPP_INCLUDE_DIRS "" CACHE STRING "Include directory for terminalpp")
set(TERMINALPP_LIBRARY "" CACHE STRING "Path to terminalpp library file")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

if(!UNIX)
add_compile_definitions(_SILENCE_CXX17_ITERATOR_BASE_CLASS_DEPRECATION_WARNING)
endif()

if(UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wno-unknown-pragmas -std=c++17 -pthread -lstdc++ -lstdc++fs -lm")
    SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -lstdc++fs" )
endif()


add_subdirectory(PotreeConverter)

if (MSVC_VERSION GREATER_EQUAL "1900")
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("/std:c++latest" _cpp_latest_flag_supported)
    if (_cpp_latest_flag_supported)
        add_compile_options("/std:c++latest")
    endif()
endif()
