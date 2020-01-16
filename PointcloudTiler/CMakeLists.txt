project(PointcloudTiler)

set(RAPIDJSON_INCLUDE_DIR lib/rapidjson/include)
set(ARGUMENTS_INCLUDE_DIR lib/arguments)

file(GLOB_RECURSE files src/*.cpp include/*.h include/*.hpp )
file(GLOB_RECURSE test_files test/*.h test/*.cpp)
file(GLOB_RECURSE lib_rapidjson_files ${RAPIDJSON_INCLUDE_DIR}/* )

set(lib_tl_expected_files ${TL_EXPECTED_INCLUDE_DIRS}/expected.hpp)

message(STATUS "LASzip include directory: ${LASZIP_INCLUDE_DIRS}")
message(STATUS "LASzip library:  ${LASZIP_LIBRARY}")

include_directories(include ${LASZIP_INCLUDE_DIRS} ${RAPIDJSON_INCLUDE_DIR} ${TL_EXPECTED_INCLUDE_DIRS})

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

#external dependencies through hunter
hunter_add_package(PROJ4)
find_package(PROJ4 CONFIG REQUIRED)

hunter_add_package(Microsoft.GSL)
find_package(Microsoft.GSL CONFIG REQUIRED)

hunter_add_package(glm)
find_package(glm REQUIRED)

hunter_add_package(Boost COMPONENTS system program_options iostreams)
find_package(Boost CONFIG REQUIRED system program_options iostreams)

add_executable(PointcloudTiler ${files} ${lib_rapidjson_files} ${lib_tl_expected_files})

if(UNIX)
	target_link_libraries(PointcloudTiler 
		PUBLIC 	${LASZIP_LIBRARY} 
				proj 
				Microsoft.GSL::GSL 
				Boost::program_options
				Boost::system
				Boost::iostreams
		PRIVATE 
				glm 
				-lstdc++fs)
else()
	target_link_libraries(PointcloudTiler 
		PUBLIC 
				${LASZIP_LIBRARY} 
				proj 
				Microsoft.GSL::GSL 
				Boost::program_options
				Boost::system 
				Boost::iostreams
		PRIVATE 
				glm)
endif()

# filter the main file from the 'files' list so that test build doesn't get multiple main definitions
set(files_without_main ${files})
list(FILTER files_without_main EXCLUDE REGEX "main.cpp")
add_executable(PointcloudTilerTest ${files_without_main} ${test_files} ${lib_rapidjson_files} ${lib_tl_expected_files})

if(UNIX)
	target_link_libraries(PointcloudTilerTest  
		PUBLIC 	${LASZIP_LIBRARY} 
				proj 
				Microsoft.GSL::GSL 
				Boost::program_options
				Boost::system 
				Boost::iostreams
		PRIVATE 
				glm 
				-lstdc++fs)
else()
	target_link_libraries(PointcloudTilerTest 
		PUBLIC 
				${LASZIP_LIBRARY} 
				proj 
				Microsoft.GSL::GSL
				Boost::program_options
				Boost::system 
				Boost::iostreams
		PRIVATE 
				glm)
endif()