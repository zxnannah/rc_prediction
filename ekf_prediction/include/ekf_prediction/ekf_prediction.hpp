#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <fstream>
#include <sstream>
#include <string>

using namespace std::chrono_literals;

class PosiRead : public rclcpp::Node
{
public:
    PosiRead();

private:
    void timer_callback();
    void ek_filter();

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::ifstream csv_file_;
    std::string file_path_;
    int line_number_;
};