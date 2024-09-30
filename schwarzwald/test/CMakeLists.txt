project(SchwarzwaldTest)

Include(FetchContent)
FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.7.1
)
FetchContent_MakeAvailable(Catch2)

set(SOURCE_FILES
    TestAlgorithm.cpp
    TestBinaryPersistence.cpp
    TestChunkRange.cpp
    TestJournal.cpp
    TestLASFile.cpp
    TestLASPersistence.cpp
    TestLRUCache.cpp
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

target_link_libraries(SchwarzwaldTest PUBLIC SchwarzwaldCore PRIVATE Catch2::Catch2WithMain)