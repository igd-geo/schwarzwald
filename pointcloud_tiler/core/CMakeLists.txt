project(PointcloudTilerCore)

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

    util/Definitions.h
    util/Error.h
    util/Error.cpp
    util/ExecutorObserver.h
    util/Stats.cpp
    util/Stats.h
    util/stuff.cpp
    util/stuff.h
    util/Transformation.cpp
    util/Transformation.h
    util/Version.h
)
set(lib_tl_expected_files ${TL_EXPECTED_INCLUDE_DIRS}/expected.hpp)

#external dependencies through hunter
hunter_add_package(PROJ4)
find_package(PROJ4 CONFIG REQUIRED)

hunter_add_package(Microsoft.GSL)
find_package(Microsoft.GSL CONFIG REQUIRED)

hunter_add_package(glm)
find_package(glm REQUIRED)

hunter_add_package(Boost COMPONENTS system program_options iostreams)
find_package(Boost CONFIG REQUIRED system program_options iostreams)

hunter_add_package(RapidJSON)
find_package(RapidJSON CONFIG REQUIRED)

add_library(PointcloudTilerCore STATIC ${SOURCE_FILES} ${lib_tl_expected_files})

target_include_directories(PointcloudTilerCore PUBLIC . include ${LASZIP_INCLUDE_DIRS} ${TL_EXPECTED_INCLUDE_DIRS})

if(UNIX)
	target_link_libraries(PointcloudTilerCore 
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
				-lstdc++fs)
else()
	target_link_libraries(PointcloudTilerCore 
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
				glm)
endif()