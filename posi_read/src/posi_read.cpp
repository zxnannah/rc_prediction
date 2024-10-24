#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "posi_read/posi_read.hpp"


BallCsvReaderNode::BallCsvReaderNode() : Node("posi_read") {
    publisher_ = this->create_publisher<geometry_msgs::msg::Point>("ball_position", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&BallCsvReaderNode::publishNextPoint, this)
    );
    loadCsvData("src/posi_read/data/data.csv");
} // 测试阶段

void BallCsvReaderNode::loadCsvData(const std::string& file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open file");
        rclcpp::shutdown();
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        double x, y, z;

        if (ss >> x && ss.get() == ',' && ss >> y && ss.get() == ',' && ss >> z) {
            data_.push_back({x, y, z});
        } else {
            RCLCPP_WARN(this->get_logger(), "Skipping malformed line: %s", line.c_str());
        }
    }
}

void BallCsvReaderNode::publishNextPoint() {
    if (index_ >= data_.size()) {
        RCLCPP_INFO(this->get_logger(), "End of CSV data reached.");
        rclcpp::shutdown();
        return;
    }

    geometry_msgs::msg::Point msg;
    msg.x = data_[index_].x;
    msg.y = data_[index_].y;
    msg.z = data_[index_].z;

    publisher_->publish(msg);

    index_++;
}
