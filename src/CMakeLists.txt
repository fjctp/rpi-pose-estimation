# add_subdirectory (external/RTIMULib/RTIMULib)
add_subdirectory (external/pigpio_i2c/src/LSM9DS1)

SET(RPI_POSE_EST_SRCS
    main.cpp
    EKF.cpp
    PoseEstimator.cpp
)

SET(EIGEN3_INCLUDE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/external/eigen
)

# SET(RTIMULIB_INCLUDE_DIR
#     ${CMAKE_CURRENT_SOURCE_DIR}/external/RTIMULib/RTIMULib
#     ${CMAKE_CURRENT_SOURCE_DIR}/external/RTIMULib/RTIMULib/IMUDrivers
# )

add_executable (rpiPoseEst ${RPI_POSE_EST_SRCS})
# target_link_libraries (rpiPoseEst RTIMULib)
target_link_libraries (rpiPoseEst lsm9ds1)
target_link_libraries (rpiPoseEst pthread)

target_include_directories (rpiPoseEst PUBLIC ${EIGEN3_INCLUDE_DIR})
# target_include_directories (rpiPoseEst PUBLIC ${RTIMULIB_INCLUDE_DIR})
target_include_directories (rpiPoseEst PUBLIC ${LSM9DS1_INCLUDE_DIR})
target_include_directories (rpiPoseEst PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
