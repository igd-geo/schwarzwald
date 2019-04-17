project(PotreeConverter)

set(RAPIDJSON_INCLUDE_DIR lib/rapidjson/include)
set(ARGUMENTS_INCLUDE_DIR lib/arguments)

file(GLOB_RECURSE files src/*.cpp include/*.h include/*.hpp )
file(GLOB_RECURSE lib_rapidjson_files ${RAPIDJSON_INCLUDE_DIR}/* )
file(GLOB_RECURSE lib_argument_files ${ARGUMENTS_INCLUDE_DIR}/* )

message(STATUS "LASzip include directory: ${LASZIP_INCLUDE_DIRS}")
message(STATUS "LASzip library:  ${LASZIP_LIBRARY}")

include_directories(include ${LASZIP_INCLUDE_DIRS})
include_directories(include ${RAPIDJSON_INCLUDE_DIR})
include_directories(include ${ARGUMENTS_INCLUDE_DIR})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/build/Debug)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${CMAKE_SOURCE_DIR}/build/RelWithDebInfo)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/build/Release)

SOURCE_GROUP("Header Files\\io" FILES 
	include/BINPointReader.hpp 
	include/BINPointWriter.hpp
	include/LASPointReader.h
	include/LASPointWriter.hpp
	include/PlyPointReader.h
	include/PointReader.h
	include/PointWriter.hpp
	include/PTXPointReader.h
	include/XYZPointReader.hpp
	include/TileMetaWriter.h
)

SOURCE_GROUP("Source Files\\io" FILES 
	src/BINPointReader.cpp
	src/LASPointReader.cpp
	src/LASPointWriter.cpp
	src/PTXPointReader.cpp
	src/TileMetaWriter.cpp
)

SOURCE_GROUP("lib\\rapidjson" FILES ${lib_rapidjson_files})
SOURCE_GROUP("lib\\arguments" FILES ${lib_argument_files})

#external dependencies through hunter
hunter_add_package(PROJ4)
find_package(PROJ4 CONFIG REQUIRED)

hunter_add_package(Microsoft.GSL)
find_package(Microsoft.GSL CONFIG REQUIRED)

hunter_add_package(glm)
find_package(glm REQUIRED)

add_executable(PotreeConverter ${files} ${lib_rapidjson_files} ${lib_argument_files})

if(UNIX)
    target_link_libraries(PotreeConverter PUBLIC ${LASZIP_LIBRARY} proj Microsoft.GSL::GSL PRIVATE glm -lstdc++fs)
else()
	target_link_libraries(PotreeConverter PUBLIC ${LASZIP_LIBRARY} proj Microsoft.GSL::GSL PRIVATE glm)
endif()
