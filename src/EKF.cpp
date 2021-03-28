#include "EKF.hpp"
#include "Eigen/Dense"

#include <cmath>

EKF::EKF() {}

EKF::~EKF() {}

void EKF::init(const double dt = 0.02) {
    _dt = dt;

    _A = Eigen::MatrixXd::Identity(3, 3);

    _x = Eigen::Vector3d(0.0, 0.0, 0.0);
    _P = 1e-6 * Eigen::MatrixXd::Identity(3, 3);

    
    double q1 = pow(0.002164253, 2.0);
    double q2 = pow(0.001151233, 2.0);
    double q3 = pow(0.001192117, 2.0);
    _Q = Eigen::DiagonalMatrix<double, 3> (q1, q2, q3);

    double r1 = pow(0.015071111, 2.0);
    double r2 = pow(0.014420412, 2.0);
    double r3 = pow(0.015951925, 2.0);
    _R = Eigen::DiagonalMatrix<double, 3> (r1, r2, r3);
}

Eigen::Matrix3d EKF::body2euler(const Eigen::Vector3d euler) {
    double phi = euler[0];
    double theta = euler[1];
    double psi = euler[2];

    Eigen::Matrix3d dcm;
    dcm <<  1, sin(phi)*tan(theta), cos(phi)*tan(theta),
            0,            cos(phi),           -sin(phi),
            0, sin(phi)/cos(theta), cos(phi)/cos(theta);
    return dcm;
}

Eigen::MatrixXd EKF::Bj(void) {
    return _dt*body2euler(_x);
}

Eigen::MatrixXd EKF::Cj(void) {
    float phi = _x[0];
    float theta = _x[1];
    float psi = _x[2];

    float g = 9.81;
    Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(3, 3);
    mat(0,1) = -cos(theta);
    mat(1,0) = cos(phi)*cos(theta);
    mat(1,1) = -sin(phi)*sin(theta);
    mat(2,0) = -sin(phi)*cos(theta);
    mat(2,1) = -cos(phi)*sin(theta);
    return g*mat;
}

void EKF::predict(Eigen::VectorXd u) {
    _x = _A*_x + body2euler(_x)*_dt*u;

    _P = _A*_P*_A.transpose() + _Q;
}

void EKF::correct(Eigen::VectorXd z) {
    double phi = _x[0];
    double theta = _x[1];
    double psi = _x[2];

    double g = 9.81;
    Eigen::Vector3d z_est;
    z_est << -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta);
    z_est *= g;
    Eigen::VectorXd y = z - z_est;

    Eigen::MatrixXd H = Cj();
    Eigen::MatrixXd S = H*_P*H.transpose() + _R;
    Eigen::MatrixXd K = _P*H.transpose()*S.inverse();

    _x += K*y;
    _P *= Eigen::MatrixXd::Identity(K.rows(),H.cols())-K*H;
}