#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class RobotMotion : public rclcpp::Node {

public:

    RobotMotion() : Node("motion") {

        // PUBLISHERS
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // SUBSCRIBERS
        size_error_sub = this->create_subscription<std_msgs::msg::Float64>("/size_error", 10, std::bind(&RobotMotion::size_error_callback, this, _1));

        // PARAMETERS
        /*this->declare_parameter<double>("k_v", 0.005);
        this->declare_parameter<double>("k_w", 0.005);

        k_v = this->get_parameter("k_v").as_double();
        k_w = this->get_parameter("k_w").as_double();*/

        RCLCPP_INFO(this->get_logger(), "Robot motion node started!");
    }

private:

    // SIZE ERROR CALLBACK
    void size_error_callback(const std_msgs::msg::Float64::SharedPtr msg) {

        geometry_msgs::msg::Twist vel;

        if (std::abs(msg->data) < 1.0) {

            vel.linear.x = 0.0;
            vel.angular.z = 0.0;

        } else {

            vel.linear.x = k_v * msg->data;
            vel.angular.z = 0.0;
        }

        cmd_vel_pub_->publish(vel);
    }

    // PUBLISHERS
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // SUBSCRIBERS
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr size_error_sub;

    // PARAMETERS
    double k_v=0.002;
    double k_w=0.002;

    // TIMERS
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<RobotMotion>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}