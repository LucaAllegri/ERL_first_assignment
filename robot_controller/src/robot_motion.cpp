
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class RobotMotion : public rclcpp::Node{

    public:

        RobotMotion() : Node("robot_motion"){
            // PUBLISHERS
            cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

            // TIMER
            timer_ = this->create_wall_timer(100ms,std::bind(&RobotMotion::moveRobot, this));

            RCLCPP_INFO(this->get_logger(),"Robot motion node started!");
        }

    private:

        void moveRobot(){
            geometry_msgs::msg::Twist cmd;

            // Move forward
            cmd.linear.x = 0.1;
            cmd.angular.z = 0.0;

            cmd_vel_pub_->publish(cmd);
        
        }

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

        rclcpp::TimerBase::SharedPtr timer_;

        int counter_ = 0;
};


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RobotMotion>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
