project(Schwarzwald)

set(SOURCE_FILES

    main.cpp

)

add_executable(Schwarzwald ${SOURCE_FILES})

target_link_libraries(Schwarzwald PUBLIC SchwarzwaldCore)