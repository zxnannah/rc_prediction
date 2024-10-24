#include "ekf_prediction/ekf_filter.hpp"

EKF::EKF(double dt) : dt_(dt) {
    x_ = VectorXd::Zero(6);  // [x, vx, y, vy, z, vz]
    P_ = MatrixXd::Identity(6, 6) * 1.0;
    Q_ = MatrixXd::Identity(6, 6) * 0.01;
    R_ = MatrixXd::Identity(3, 3) * 0.1;
    F_ = MatrixXd::Identity(6, 6);
    F_(0, 1) = dt_;
    F_(2, 3) = dt_;
    F_(4, 5) = dt_;

    g_ = -9.81;
    B_ = VectorXd::Zero(6);
    B_(5) = g_ * dt_;

    H_ = MatrixXd::Zero(3, 6);
    H_(0, 0) = 1;  // x
    H_(1, 2) = 1;  // y
    H_(2, 4) = 1;  // z
}

void EKF::predict() {
    x_ = F_ * x_ + B_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void EKF::update(const VectorXd& z) {
    MatrixXd S = H_ * P_ * H_.transpose() + R_;
    MatrixXd K = P_ * H_.transpose() * S.inverse(); //增益

    VectorXd y = z - H_ * x_;
    x_ = x_ + K * y;

    P_ = (MatrixXd::Identity(6, 6) - K * H_) * P_;
}

VectorXd EKF::getState() const {
    return x_;
}

void EKF::setProcessNoise(double q) {
    Q_ = MatrixXd::Identity(6, 6) * q;
}

void EKF::setMeasurementNoise(double r) {
    R_ = MatrixXd::Identity(3, 3) * r;
}
