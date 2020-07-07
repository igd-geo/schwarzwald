project(util)

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

set(files 
	algorithms/Algorithm.h
	algorithms/Enums.h
	algorithms/Pairs.h
	algorithms/Strings.h

	concepts/MemoryIntrospection.h

	containers/DestructuringIterator.h
	
	debug/Journal.h
	debug/Journal.cpp
	debug/ProgressReporter.h
	debug/ThroughputCounter.h
	debug/ThroughputCounter.cpp
	debug/Timing.h

	io/io_util.h
	io/io_util.cpp

	terminal/stdout_helper.h
	terminal/stdout_helper.cpp
	terminal/TerminalUI.h
	terminal/TerminalUI.cpp

	threading/Async.h
	threading/Async.cpp
	threading/Parallel.h
	threading/Semaphore.h
	threading/Semaphore.cpp
	threading/TaskSystem.h
	threading/TaskSystem.cpp

	types/type_util.h
	types/Units.h
	types/Units.cpp
)

set(lib_tl_expected_files ${TL_EXPECTED_INCLUDE_DIRS}/expected.hpp)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/build/Debug)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${CMAKE_SOURCE_DIR}/build/RelWithDebInfo)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/build/Release)

#external dependencies through hunter
hunter_add_package(Microsoft.GSL)
find_package(Microsoft.GSL CONFIG REQUIRED)

hunter_add_package(Boost COMPONENTS system program_options iostreams)
find_package(Boost CONFIG REQUIRED system program_options iostreams)

add_library(util STATIC ${files} ${lib_tl_expected_files})

target_include_directories(util PUBLIC . ${TL_EXPECTED_INCLUDE_DIRS} ${cpp-taskflow_SOURCE_DIR})

if(UNIX)
	target_link_libraries(util 
		PUBLIC 	
				Microsoft.GSL::GSL 
				Boost::program_options
				Boost::system
				Boost::iostreams
		PRIVATE 
				-lstdc++fs)
else()
	target_link_libraries(PointcloudTiler 
		PUBLIC 
				Microsoft.GSL::GSL 
				Boost::program_options
				Boost::system 
				Boost::iostreams)
endif()