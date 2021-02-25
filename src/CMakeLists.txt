add_subdirectory (external/RTIMULib/RTIMULib)

SET(RPI_POSE_EST_SRCS
    main.cpp
    EKF.cpp
    PoseEstimator.cpp
)

SET(EIGEN3_INCLUDE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/external/eigen
)

SET(RTIMULIB_INCLUDE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/external/RTIMULib/RTIMULib
    ${CMAKE_CURRENT_SOURCE_DIR}/external/RTIMULib/RTIMULib/IMUDrivers
)

add_executable (rpiPoseEst ${RPI_POSE_EST_SRCS})
target_link_libraries (rpiPoseEst RTIMULib)

target_include_directories (rpiPoseEst PUBLIC ${EIGEN3_INCLUDE_DIR})
target_include_directories (rpiPoseEst PUBLIC ${RTIMULIB_INCLUDE_DIR})
target_include_directories (rpiPoseEst PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
