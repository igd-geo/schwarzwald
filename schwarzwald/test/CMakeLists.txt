project(SchwarzwaldTest)

set(SOURCE_FILES

    catch.hpp

    TestAlgorithm.cpp
    TestBinaryPersistence.cpp
    TestChunkRange.cpp
    TestJournal.cpp
    TestLASFile.cpp
    TestLASPersistence.cpp
    TestLRUCache.cpp
    TestMain.cpp
    TestMemoryIntrospection.cpp
    TestMortonIndex.cpp
    TestOctree.cpp
    TestOctreeIndexing.cpp
    TestOctreeIndexWriter.cpp
    TestOctreeNodeIndex.cpp
    TestTiler.cpp
    TestUnits.cpp
    TestUtilities.cpp
)

add_executable(SchwarzwaldTest ${SOURCE_FILES})

target_link_libraries(SchwarzwaldTest PUBLIC SchwarzwaldCore)