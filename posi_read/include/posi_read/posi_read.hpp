#ifndef POSI_READ_HPP_
#define POSI_READ_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

class BallCsvReaderNode : public rclcpp::Node {
public:
    BallCsvReaderNode();

private:
    void loadCsvData(const std::string& file_path);

    void publishNextPoint();
    struct Point3D {
        double x, y, z;
    };

    std::vector<Point3D> data_;
    size_t index_ = 0;

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif
