add_library(imu_support OBJECT imu_support.c)
target_include_directories(imu_support PUBLIC include)
target_link_libraries(imu_support PRIVATE zephyr_interface)

target_sources(app PRIVATE $<TARGET_OBJECTS:imu_support>)
