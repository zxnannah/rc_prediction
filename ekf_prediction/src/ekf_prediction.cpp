#include "ekf_prediction/ekf_filter.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath> 
#include <string>

class EKFNode : public rclcpp::Node {
public:
    EKFNode() : Node("ekf_node"), ekf_(0.033) {  // 假设识别频率1秒30帧
        subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "ball_position", 10, 
            std::bind(&EKFNode::positionCallback, this, std::placeholders::_1)
        );

        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("filtered_position", 10);
        landing_pub_ = this->create_publisher<geometry_msgs::msg::Point>("landing_position", 10);

        // 初始化参数，并绑定参数回调函数
        this->declare_parameter("process_noise", 0.01);  // 初始 q 值，q增大信任观测值
        this->declare_parameter("measurement_noise", 0.1);  // 初始 r 值，r增大信任预测值

        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&EKFNode::parameterCallback, this, std::placeholders::_1)
        );
    }

private:
    void positionCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        VectorXd z(3);
        z << msg->x, msg->y, msg->z;

        ekf_.predict();
        ekf_.update(z);

        VectorXd state = ekf_.getState();
        geometry_msgs::msg::Point filtered_msg;
        filtered_msg.x = state(0);  // 滤波后x
        filtered_msg.y = state(2);  // 滤波后y
        filtered_msg.z = state(4);  // 滤波后z
        publisher_->publish(filtered_msg);

        double landing_x = predictLandingPoint_X(state);
        double landing_y = predictLandingPoint_Y(state);
        geometry_msgs::msg::Point landing_msg;
        landing_msg.x = landing_x;
        landing_msg.y = landing_y;
        landing_msg.z = 0.0;  // 落地
        landing_pub_->publish(landing_msg);

        RCLCPP_INFO(this->get_logger(), 
                    "Filtered Position: [x=%.2f, y=%.2f, z=%.2f], Predicted Landing: [x=%.2f, y=%.2f]",
                    state(0), state(2), state(4), landing_x,landing_y); // 调好了就注掉
    }

    rcl_interfaces::msg::SetParametersResult parameterCallback(
        const std::vector<rclcpp::Parameter>& parameters) {

        for (const auto& param : parameters) {
            if (param.get_name() == "process_noise") {
                double q = param.as_double();
                ekf_.setProcessNoise(q);
                RCLCPP_INFO(this->get_logger(), "----------------- q 更新: %.4f -----------------", q);
            } else if (param.get_name() == "measurement_noise") {
                double r = param.as_double();
                ekf_.setMeasurementNoise(r);
                RCLCPP_INFO(this->get_logger(), "----------------- r 更新: %.4f -----------------", r);
            }
        }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }  //  动态调参写的，但现在数据太短不好调，等以后的自适应调参吧

    double predictLandingPoint_X(const VectorXd& state) {
        double x0 = state(0);  // x 位置
        double vx0 = state(1);  // x 速度
        double z0 = state(4);  // z 位置
        double vz0 = state(5);  // z 速度

        double g = 9.81;
        double discriminant = vz0 * vz0 + 2 * g * z0;

        if (discriminant < 0) {
            RCLCPP_WARN(this->get_logger(), "！！！！！！！！！无数据！！！！！！！！");
            return std::numeric_limits<double>::quiet_NaN();
        }

        double tf = (-vz0 - std::sqrt(discriminant)) / g;
        double xf = x0 + vx0 * tf;
        return xf;
    }

    double predictLandingPoint_Y(const VectorXd& state) {
        double y0 = state(2);  //y 位置
        double vy0 = state(3);  //y 速度
        double z0 = state(4);  // z 位置
        double vz0 = state(5);  // z 速度

        double g = 9.81;
        double discriminant = vz0 * vz0 + 2 * g * z0;

        if (discriminant < 0) {
            RCLCPP_WARN(this->get_logger(), "！！！！！！！！！无数据！！！！！！！！");
            return std::numeric_limits<double>::quiet_NaN();
        }

        double tf = (-vz0 - std::sqrt(discriminant)) / g;
        double yf = y0 + vy0 * tf;
        return yf;
    }  //脑子没有了等有脑子的时候把x,y放一起预测，现在先这样

    EKF ekf_; 
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr landing_pub_;  //发出去了，其他的后续写
    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

// 注册节点
// #include "rclcpp_components/register_node_macro.hpp"
// RCLCPP_COMPONENTS_REGISTER_NODE(EKFNode)
