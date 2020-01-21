project(PointcloudTiler)

FetchContent_Declare(
	cpp-taskflow
	GIT_REPOSITORY https://github.com/cpp-taskflow/cpp-taskflow.git
	GIT_TAG v2.2.0
)
set(TF_BUILD_EXAMPLES FALSE)
set(TF_BUILD_TESTS FALSE)
set(TF_BUILD_BENCHMARKS FALSE)

FetchContent_GetProperties(cpp-taskflow)
if(NOT cpp-taskflow_POPULATED)
  FetchContent_Populate(cpp-taskflow)
  add_subdirectory(${cpp-taskflow_SOURCE_DIR} ${cpp-taskflow_BINARY_DIR})
endif()

set(RAPIDJSON_INCLUDE_DIR lib/rapidjson/include)

file(GLOB_RECURSE files src/*.cpp include/*.h include/*.hpp )
file(GLOB_RECURSE test_files test/*.h test/*.cpp)
file(GLOB_RECURSE lib_rapidjson_files ${RAPIDJSON_INCLUDE_DIR}/* )

set(lib_tl_expected_files ${TL_EXPECTED_INCLUDE_DIRS}/expected.hpp)

message(STATUS "LASzip include directory: ${LASZIP_INCLUDE_DIRS}")
message(STATUS "LASzip library:  ${LASZIP_LIBRARY}")

include_directories(include ${LASZIP_INCLUDE_DIRS} ${RAPIDJSON_INCLUDE_DIR} ${TL_EXPECTED_INCLUDE_DIRS} ${cpp-taskflow_SOURCE_DIR})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/build/Debug)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${CMAKE_SOURCE_DIR}/build/RelWithDebInfo)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/build/Release)

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