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
    // 构造函数声明
    BallCsvReaderNode();

private:
    // CSV 读取函数声明
    void loadCsvData(const std::string& file_path);

    // 发布数据点的函数声明
    void publishNextPoint();

    // 3D 点结构体
    struct Point3D {
        double x, y, z;
    };

    // 数据存储向量
    std::vector<Point3D> data_;
    size_t index_ = 0;  // 当前读取的索引

    // ROS2 发布者和定时器
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif
