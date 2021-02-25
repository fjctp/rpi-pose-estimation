#ifndef EKF_HPP
#define EKF_HPP

#include "Eigen/Dense"

class EKF {
    private:
        double _dt;

        Eigen::MatrixXd _A;
        
        Eigen::MatrixXd _Q;
        Eigen::MatrixXd _R;

        Eigen::VectorXd _x;
        Eigen::MatrixXd _P;

    public:
        EKF();
        ~EKF();
        void init(double dt);
        void predict(Eigen::VectorXd u);
        void correct(Eigen::VectorXd z);

        Eigen::Matrix3d body2euler(const Eigen::Vector3d euler);
        Eigen::MatrixXd Bj(void);
        Eigen::MatrixXd Cj(void);

        Eigen::VectorXd getStates(void) {return _x;}
};

#endif