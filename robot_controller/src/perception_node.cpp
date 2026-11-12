
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_interfaces/msg/markerImage.hpp"

using namespace std::chrono_literals;

class PerceptionNode : public rclcpp::Node{

    public:

        RobotMotion() : Node("perception_node"){

            // PUBLISHERS
            marker_img_pub = this->create_publisher<robot_interfaces::msg::MarkerImage>("/marker_image", 10);

            // SUBSCRIBERS
            aruco_detection_sub = this->create_subscriber<aruco_opencv_msgs::msg::ArucoDetection>("/aruco_detections", 10);

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
