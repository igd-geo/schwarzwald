project(SchwarzwaldCore)

set(SOURCE_FILES
    datastructures/GridCell.h
    datastructures/GridCell.cpp
    datastructures/GridIndex.h
    datastructures/LRUCache.h
    datastructures/PointBuffer.h
    datastructures/PointBuffer.cpp
    datastructures/SparseGrid.h
    datastructures/SparseGrid.cpp
    datastructures/DynamicMortonIndex.cpp
    datastructures/DynamicMortonIndex.h
    datastructures/MortonIndex.h
    datastructures/MortonGrid.cpp
    datastructures/MortonGrid.h

    io/BinaryPersistence.cpp
    io/BinaryPersistence.h
    io/Cesium3DTilesPersistence.cpp
    io/Cesium3DTilesPersistence.h
    io/LASFile.cpp
    io/LASFile.h
    io/LASPersistence.cpp
    io/LASPersistence.h
    io/EntwinePersistence.cpp
    io/EntwinePersistence.h
    io/MemoryPersistence.cpp
    io/MemoryPersistence.h
    io/PNTSReader.cpp
    io/PNTSReader.h
    io/PNTSWriter.cpp
    io/PNTSWriter.h
    io/PointcloudFactory.cpp
    io/PointcloudFactory.h
    io/PointcloudFile.h
    io/PointReader.h
    io/PointsPersistence.cpp
    io/PointsPersistence.h
    io/TileSetWriter.cpp
    io/TileSetWriter.h

    math/AABB.h
    math/Vector3.h

    tiling/Node.cpp
    tiling/Node.h
    tiling/OctreeAlgorithms.cpp
    tiling/OctreeAlgorithms.h
    tiling/OctreeIndexWriter.h
    tiling/Sampling.cpp
    tiling/Sampling.h
    tiling/TilingAlgorithms.h
    tiling/TilingAlgorithms.cpp

    pointcloud/FileStats.h
    pointcloud/FileStats.cpp
    pointcloud/Point.h
    pointcloud/PointAttributes.cpp
    pointcloud/PointAttributes.h
    pointcloud/Tileset.cpp
    pointcloud/Tileset.h

    point_source/PointSource.cpp
    point_source/PointSource.h

    process/ConverterProcess.cpp
    process/ConverterProcess.h
    process/Tiler.cpp
    process/Tiler.h
    process/TilerProcess.cpp
    process/TilerProcess.h

    util/Config.h
    util/Config.cpp
    util/Definitions.h
    util/Error.h
    util/Error.cpp
    util/Scheduler.h
    util/Scheduler.cpp
    util/Stats.cpp
    util/Stats.h
    util/stuff.cpp
    util/stuff.h
    util/Transformation.cpp
    util/Transformation.h
    util/Version.h
)
set(lib_tl_expected_files ${TL_EXPECTED_INCLUDE_DIRS}/expected.hpp)

#external dependencies

include(FetchContent)
FetchContent_Declare(proj
  URL https://github.com/OSGeo/PROJ/releases/download/9.5.0/proj-9.5.0.tar.gz
  URL_HASH SHA1=77782c889ca727eaf4696055604634b4d253f9a0
)
cmake_policy(PUSH)
cmake_policy(SET CMP0077 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
set(BUILD_TESTING_OLD ${BUILD_TESTING})
set(ENABLE_TIFF_OLD ${ENABLE_TIFF})
set(BUILD_APPS_OLD ${BUILD_APPS})
set(BUILD_TESTING OFF)
set(ENABLE_TIFF OFF)
set(BUILD_APPS OFF)
FetchContent_MakeAvailable(proj)
set(BUILD_TESTING ${BUILD_TESTING_OLD})
set(ENABLE_TIFF ${ENABLE_TIFF_OLD})
set(BUILD_APPS ${BUILD_APPS_OLD})
cmake_policy(POP)

hunter_add_package(Microsoft.GSL)
find_package(Microsoft.GSL CONFIG REQUIRED)

hunter_add_package(glm)
find_package(glm REQUIRED)

#hunter_add_package(Boost COMPONENTS system program_options iostreams)
find_package(Boost CONFIG REQUIRED system program_options iostreams)

hunter_add_package(RapidJSON)
find_package(RapidJSON CONFIG REQUIRED)

find_package(Threads REQUIRED)

add_library(SchwarzwaldCore STATIC ${SOURCE_FILES} ${lib_tl_expected_files})

target_include_directories(SchwarzwaldCore PUBLIC . include ${LASZIP_INCLUDE_DIRS} ${TL_EXPECTED_INCLUDE_DIRS})

if(UNIX)
	target_link_libraries(SchwarzwaldCore
        PUBLIC 	util
                ${LASZIP_LIBRARY}
				proj
				Microsoft.GSL::GSL
				Boost::program_options
				Boost::system
                Boost::iostreams
                RapidJSON::rapidjson
		PRIVATE
				glm
                Threads::Threads
				-lstdc++fs)
else()
	target_link_libraries(SchwarzwaldCore
        PUBLIC
                util
				${LASZIP_LIBRARY}
				proj
				Microsoft.GSL::GSL
				Boost::program_options
				Boost::system
                Boost::iostreams
                RapidJSON::rapidjson
		PRIVATE
				glm
                Threads::Threads)
endif()
