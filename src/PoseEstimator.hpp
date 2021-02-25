#include "EKF.hpp"
#include "Eigen/Dense"

// see https://pinout.xyz/pinout/sense_hat
#include "RTIMULib.h"
#include "RTIMULSM9DS1.h" // 9-axis IMU

class PoseEstimator {
    private: 
        EKF _filter;
        
        RTIMUSettings *_settings;
        RTIMU *_imu;

    public:
        PoseEstimator();
        ~PoseEstimator();

        void init(const double dt);
        void step(void);
        Eigen::Vector3d getEuler(void);
};