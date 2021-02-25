##
SET(TEST_EKF_SRCS
    test_EKF.cpp
    ../EKF.cpp
)

SET(EIGEN3_INCLUDE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/../external/eigen
)

add_executable (test_EKF ${TEST_EKF_SRCS})

target_include_directories (test_EKF PUBLIC ${EIGEN3_INCLUDE_DIR})
target_include_directories (test_EKF PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories (test_EKF PUBLIC ..)

# ##
# SET(TEST_POSEEST_SRCS
#     test_PoseEst.cpp
#     ../PoseEstimator.cpp
# )

# add_executable (test_PoseEstimator ${TEST_POSEEST_SRCS})

# target_include_directories (test_PoseEstimator PUBLIC ${EIGEN3_INCLUDE_DIR})
# target_include_directories (test_PoseEstimator PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
# target_include_directories (test_PoseEstimator PUBLIC ..)