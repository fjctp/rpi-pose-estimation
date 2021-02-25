#include "EKF.hpp"
#include "Eigen/Dense"

#include <iostream>

int main(void) {
    std::cout << "Testing EKF" << std::endl;

    double dt = 1.0/100.0;
    Eigen::Vector3d pqr(0.017, 0.017, 0.017);
    Eigen::Vector3d ab(0.1, 0.1, 0.9899);

    ab *= 9.81;

    EKF filter;

    filter.init(dt);
    std::cout << "x0 = " << filter.getStates().transpose() << std::endl;

    std::cout << "DCM = " << filter.body2euler(Eigen::Vector3d(0.0, 0.0, 0.0)) << std::endl;
    std::cout << "Bj = " << filter.Bj() << std::endl;
    std::cout << "Cj = " << filter.Cj() << std::endl;

    for(int i = 0; i<2; i++) {
        std::cout << "Step " << i+1 << " - Predict" << std::endl;
        filter.predict(pqr);
        std::cout << "x1 = " << filter.getStates().transpose() << std::endl;

        std::cout << "Step " << i+1 << " - Correct" << std::endl;
        filter.correct(ab);
        std::cout << "x2 = " << filter.getStates().transpose() << std::endl;
    }

    return -1;
}