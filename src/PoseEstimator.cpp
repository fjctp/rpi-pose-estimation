#include "PoseEstimator.hpp"

PoseEstimator::PoseEstimator() {
    _settings = new RTIMUSettings(); // use default name (RTIMULib.ini) for settings file
}

PoseEstimator::~PoseEstimator() {

}

void PoseEstimator::init(const double dt) {
    _imu = RTIMU::createIMU(_settings);
    _imu->IMUInit();
    int interval_ms = _imu->IMUGetPollInterval();
    
    _filter.init(dt);
}

void PoseEstimator::step(void) {
    RTIMU_DATA data = _imu->getIMUData();
}

Eigen::Vector3d PoseEstimator::getEuler(void) {
    return _filter.getStates();
}