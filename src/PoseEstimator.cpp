#include "PoseEstimator.hpp"
//#include "RTIMULib.h"
#include "LSM9DS1.hpp"

#include <iostream>
#include <thread>

PoseEstimator::PoseEstimator() {
    // use default name (RTIMULib.ini) for settings file
    // _settings = new RTIMUSettings();
    // _settings->m_fusionType = 0; // disable fusion

    // _imu = NULL;

    _gyro << 0.0, 0.0, 0.0;
    _accel << 0.0, 0.0, 0.0;
    _compass << 0.0, 0.0, 0.0;

    gyro_offset << 8.044149180951079e-4, -1.045227287770315e-4, 4.720142456744947e-4;
    gyro_offset *= -1;
    accel_offset << 0.0, 0.0, 9.715670410184750;
}

PoseEstimator::~PoseEstimator() {
    // if(_settings != NULL)
    //     delete(_settings);
    // if(_imu != NULL)
    //     delete(_imu);
}

void PoseEstimator::init(const double dt) {
    // _imu = RTIMU::createIMU(_settings);
    // _imu->IMUInit();

    // _imu->setSlerpPower(0.02);
    // _imu->setGyroEnable(true);
    // _imu->setAccelEnable(true);
    // _imu->setCompassEnable(true);
    // _imu->setDebugEnable(true);

    // std::cout << _imu->IMUName() << std::endl;

    // to-do: use the bigger value: interval_ms/1000.0 vs dt.
    // int interval_ms = _imu->IMUGetPollInterval();
    //calibrate(dt);

    imu.init();
    imu.gBias[0] = 0.098456801;
    imu.gBias[1] = 0.00451882;
    imu.gBias[2] = 0.006186556;
    _filter.init(dt);
}

bool PoseEstimator::step(void) {
    // bool isNewData = _imu->IMURead();
    imu.read();
    // if(!isNewData)
    //     return isNewData;
    // RTIMU_DATA data = _imu->getIMUData();

    // _gyro << data.gyro.x(), data.gyro.y(), data.gyro.z();
    // _gyro += gyro_offset;

    // _accel << data.accel.x(), data.accel.y(), data.accel.z();
    // _accel /= accel_offset[2];

    auto gyro_readings = imu.gyro();
    auto accel_readings = imu.accel();
    _gyro << gyro_readings[0], gyro_readings[1], gyro_readings[2];
    _accel << accel_readings[0], accel_readings[1], accel_readings[2];

    // _accel *= 9.81f;
    _filter.predict(_gyro);
    _filter.correct(_accel);

    // return isNewData;
    return true;
}

void PoseEstimator::calibrate(const double dt) {
    // using namespace std::chrono_literals;

    // const int max_sample_count = 100;

    // for(int i=0; i<max_sample_count; i++) {
    //     bool isNewData = _imu->IMURead();
    //     RTIMU_DATA data = _imu->getIMUData();
        
    //     gyro_offset += Eigen::Vector3d(data.gyro.x(), data.gyro.y(), data.gyro.z());
    //     accel_offset += Eigen::Vector3d(data.accel.x(), data.accel.y(), data.accel.z());
    //     std::this_thread::sleep_for(dt * 1000ms);
    // }

    // gyro_offset /= -max_sample_count;
    // accel_offset /= max_sample_count;
    // accel_offset = Eigen::Vector3d(0.0, 0.0, 1.0) - accel_offset;
}
