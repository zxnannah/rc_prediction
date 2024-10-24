#ifndef EKF_FILTER_HPP_
#define EKF_FILTER_HPP_

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>

using namespace Eigen;

class EKF {
public:
    EKF(double dt);
    void predict();
    void update(const VectorXd& z);
    VectorXd getState() const;
    void setProcessNoise(double q);
    void setMeasurementNoise(double r);

private:
    double dt_;
    VectorXd x_;
    MatrixXd P_;
    MatrixXd Q_;
    MatrixXd R_;
    MatrixXd F_;
    MatrixXd H_;
    VectorXd B_;
    double g_;
};
#endif