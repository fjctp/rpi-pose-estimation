#include <iostream>
#include "PoseEstimator.hpp"

int main(void) {
    std::cout << "Test" << std::endl;

    PoseEstimator estimator;
    estimator.init(1.0/100.0);

    bool doLoop = true;
    while(doLoop) {
        estimator.step();
    }

    return 0;
}