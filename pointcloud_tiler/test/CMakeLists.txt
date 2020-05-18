project(PointcloudTilerTest)

set(SOURCE_FILES

    catch.hpp

    TestAlgorithm.cpp
    TestBinaryPersistence.cpp
    TestChunkRange.cpp
    TestLASFile.cpp
    TestLASPersistence.cpp
    TestLRUCache.cpp
    TestMain.cpp
    TestMemoryIntrospection.cpp
    TestMortonIndex.cpp
    TestOctreeIndexing.cpp
    TestOctreeIndexWriter.cpp
    TestOctreeNodeIndex.cpp
    TestTiler.cpp
    TestUnits.cpp
    TestUtilities.cpp
)

add_executable(PointcloudTilerTest ${SOURCE_FILES})

target_link_libraries(PointcloudTilerTest PUBLIC PointcloudTilerCore)