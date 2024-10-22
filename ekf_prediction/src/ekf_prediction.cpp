#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <chrono>
#include "ekf_prediction/ekf_prediction.hpp"

using namespace std::chrono_literals;

class PosiRead_Node : public rclcpp::Node
{
public:
    PosiRead_Node()
        : Node("posi_read_node"), file_path_("./data/data.csv"), line_number_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("position_topic", 10);
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "position_topic", 10, std::bind(&PosiRead_Node::ek_filter, this, std::placeholders::_1));

        csv_file_.open(file_path_);
        if (!csv_file_.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the CSV file: %s", file_path_.c_str());
            rclcpp::shutdown();
        }

        timer_ = this->create_wall_timer(33ms, std::bind(&PosiRead_Node::publish_line, this));
    }

private:
    void publish_line()
    {
        std::string line;
        if (std::getline(csv_file_, line))
        {
            auto message = std_msgs::msg::String();
            message.data = line;

            RCLCPP_INFO(this->get_logger(), "Publishing line %d: '%s'", line_number_, line.c_str());
            publisher_->publish(message);

            line_number_++;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Reached end of CSV file. Stopping the node.");
            rclcpp::shutdown();
        }
    }

    void ek_filter(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received line in ek_filter: '%s'", msg->data.c_str());

        std::string line = msg->data;
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::ifstream csv_file_;
    std::string file_path_;
    int line_number_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PosiRead_Node>());
    rclcpp::shutdown();
    return 0;
}
