#include <atomic>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <chrono>
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

    double test_time = 90.0;
    double hz = 100.0;
    double dt = 1.0/hz;
    std::chrono::duration<double> dt_ns = dt * 1e9 *1ns;

    PoseEstimator estimator;
    estimator.init(dt);

    Eigen::Vector3d eulerEst, gyro, accel;

    for(uint32_t i = 0; i<(test_time*hz); i++) {
        auto start = std::chrono::system_clock::now();

        estimator.step();
        eulerEst = estimator.eulerEst().transpose();
        gyro = estimator.gyro().transpose();
        accel = estimator.accel().transpose();
        
        auto end = std::chrono::system_clock::now();
        auto time_used = end - start;
        auto sleep_time = dt_ns - time_used;
        if (sleep_time.count() < 0)
            sleep_time = 0 * 1ns;
        auto real_time = time_used + sleep_time;

        //system("clear");
        std::cout << std::setw(6) << i;
        std::cout << " | ";
        std::cout << std::setw(10) << std::fixed << std::setprecision(0) << end.time_since_epoch().count() / 1.0e6;
        std::cout << " | ";
        std::cout << std::setw(6) << std::fixed << std::setprecision(2) << 1e9/real_time.count();
        std::cout << " | ";
        std::cout << std::setw(10) << std::fixed << std::setprecision(6) << gyro[0] << ", ";
        std::cout << std::setw(10) << std::fixed << std::setprecision(6) << gyro[1] << ", ";
        std::cout << std::setw(10) << std::fixed << std::setprecision(6) << gyro[2];
        std::cout << " | ";
        std::cout << std::setw(8) << std::fixed << std::setprecision(4) << accel[0] << ", ";
        std::cout << std::setw(8) << std::fixed << std::setprecision(4) << accel[1] << ", ";
        std::cout << std::setw(8) << std::fixed << std::setprecision(4) << accel[2];
        std::cout << " | ";
        std::cout << std::setw(8) << std::fixed << std::setprecision(4) << eulerEst[0] * 180.0 / M_PI<< ", ";
        std::cout << std::setw(8) << std::fixed << std::setprecision(4) << eulerEst[1] * 180.0 / M_PI << ", ";
        std::cout << std::setw(8) << std::fixed << std::setprecision(4) << eulerEst[2] * 180.0 / M_PI;
        std::cout << std::endl;
        std::this_thread::sleep_for(sleep_time);
    }

    // clean up
    ioThread.join();
    fs_eulerEst.close();
    fs_gyro.close();
    fs_accel.close();

    return 0;
}
