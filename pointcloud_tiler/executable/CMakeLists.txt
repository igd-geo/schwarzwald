project(PointcloudTiler)

set(SOURCE_FILES

    main.cpp

)

add_executable(PointcloudTiler ${SOURCE_FILES})

target_link_libraries(PointcloudTiler PUBLIC PointcloudTilerCore)