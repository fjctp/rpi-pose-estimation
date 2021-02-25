#include <atomic>
#include <fstream>
#include <iostream>
#include <cstdio>
#include <thread>
#include <stdio.h>
#include "Eigen/Dense"

#include "PoseEstimator.hpp"

std::atomic<bool> doLoop;

void check_key_press(void) {
    std::cout << "Press 'q' to quit" << std::endl;
    doLoop = true;

    int c = std::getchar();

    if (c == 113 || c == 81 ) {
        doLoop = false;
    }
}

int main(void) {
    using namespace std::chrono_literals;
    std::thread ioThread (check_key_press);

    std::fstream fs_eulerEst, fs_gyro, fs_accel;
    
    fs_eulerEst.open("eulerEst.csv", std::fstream::out);
    fs_gyro.open("gyro.csv", std::fstream::out);
    fs_accel.open("accel.csv", std::fstream::out);

    double dt = 1.0/100.0;

    PoseEstimator estimator;
    estimator.init(dt);

    Eigen::Vector3d eulerEst, gyro, accel;

    while(doLoop) {
        //std::cout << "Step" << std::endl;
        estimator.step();
        eulerEst = estimator.eulerEst().transpose();
        gyro = estimator.gyro().transpose();
        accel = estimator.accel().transpose();

        fs_gyro << gyro[0] << "," << gyro[1] << "," << gyro[2] << std::endl;
        fs_accel << accel[0] << "," << accel[1] << "," << accel[2] << std::endl;
        fs_eulerEst << eulerEst[0] << "," << eulerEst[1] << "," << eulerEst[2] << std::endl;

        std::this_thread::sleep_for(dt * 1000ms);
    }

    // clean up
    ioThread.join();
    fs_eulerEst.close();
    fs_gyro.close();
    fs_accel.close();

    return 0;
}
