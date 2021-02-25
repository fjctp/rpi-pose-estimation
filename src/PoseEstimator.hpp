#ifndef POSE_ESTIMATOR_HPP
#define POSE_ESTIMATOR_HPP

#include "EKF.hpp"
#include "Eigen/Dense"
//#include "RTIMULib.h"
#include "LSM9DS1.hpp"

class PoseEstimator {
    private: 
        EKF _filter;
        
        // RTIMUSettings *_settings;
        // RTIMU *_imu;
        LSM9DS1 imu;
        
        Eigen::Vector3d _gyro;
        Eigen::Vector3d _accel;
        Eigen::Vector3d _compass;

        void calibrate(const double dt);

    public:
        Eigen::Vector3d gyro_offset;
        Eigen::Vector3d accel_offset;
        
        PoseEstimator();
        ~PoseEstimator();

        void init(const double dt);
        bool step(void);

        Eigen::Vector3d eulerEst(void) { return _filter.getStates(); };
        Eigen::Vector3d gyro(void) { return _gyro; };
        Eigen::Vector3d accel(void) { return _accel; };
        Eigen::Vector3d compass(void) { return _compass; };
};

#endif