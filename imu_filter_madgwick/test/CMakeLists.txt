if(NOT WIN32)
    ament_add_gtest(madgwick_test madgwick_test.cpp)
    target_link_libraries(madgwick_test imu_filter_madgwick)

    ament_add_gtest(stateless_orientation_test stateless_orientation_test.cpp)
    target_link_libraries(stateless_orientation_test imu_filter_madgwick)
endif()
