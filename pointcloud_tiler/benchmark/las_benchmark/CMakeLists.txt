project(LASBenchmark)

set(SOURCE_FILES LASBenchmark.cpp)

add_executable(LASBenchmark ${SOURCE_FILES})
target_link_libraries(LASBenchmark PUBLIC PointcloudTilerCore)