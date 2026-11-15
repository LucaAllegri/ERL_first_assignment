
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class RobotMotion : public rclcpp::Node{

    public:

        RobotMotion() : Node("motion"){

            //PUBLISHERS
            cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

            //SUBSCRIBER
            intermediate_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("/intermediate_cmd_vel", 10, std::bind(&RobotMotion::vel_callback, this, _1));

            RCLCPP_INFO(this->get_logger(),"Robot motion node started!");
        }

    private:

        void vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
            cmd_vel_pub_->publish(*msg);
        }

        //PUBLISHERS
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

        //SUBSCRIPTION
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr intermediate_vel_sub;

        //TIMERS
        rclcpp::TimerBase::SharedPtr timer_;

};


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RobotMotion>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
