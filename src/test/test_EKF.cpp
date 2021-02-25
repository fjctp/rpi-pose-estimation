#include "EKF.hpp"
#include "Eigen/Dense"

#include <iostream>

int main(void) {
    std::cout << "Testing EKF" << std::endl;

    double dt = 1.0/100.0;
    Eigen::Vector3d pqr(0.017, 0.017, 0.017);
    Eigen::Vector3d ab(0, 0, 9.81);

    EKF filter;

    std::cout << "x1 = " << filter.getStates().transpose() << std::endl;

    filter.init(dt);
    std::cout << "x2 = " << filter.getStates().transpose() << std::endl;

    std::cout << "DCM = " << filter.body2euler(Eigen::Vector3d(0.0, 0.0, 0.0)) << std::endl;
    std::cout << "Bj = " << filter.Bj() << std::endl;
    std::cout << "Cj = " << filter.Cj() << std::endl;

    filter.predict(pqr);
    std::cout << "x3 = " << filter.getStates().transpose() << std::endl;

    filter.correct(ab);
    std::cout << "x4 = " << filter.getStates().transpose() << std::endl;

    return -1;
}