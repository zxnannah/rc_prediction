#include <rclcpp/rclcpp.hpp>
#include "posi_read/posi_read.hpp"  // 包含头文件，而不是源文件

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BallCsvReaderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
