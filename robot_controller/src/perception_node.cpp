
#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_interfaces/msg/marker_image.hpp"
#include "aruco_opencv_msgs/msg/aruco_detection.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
using std::placeholders::_1;

using namespace std::chrono_literals;

class PerceptionNode : public rclcpp::Node{

    public:

        PerceptionNode() : Node("perception_node"){

            //PUBLISHERS
            marker_img_pub = this->create_publisher<robot_interfaces::msg::MarkerImage>("/marker_image", 10);
            intermediate_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/intermediate_cmd_vel", 10);

            //SUBSCRIBERS
            aruco_detection_sub = this->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>("/aruco_detections", 10, std::bind(&PerceptionNode::aruco_detect_callback, this, _1));
            camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>("/camera/camera_info", 10, std::bind(&PerceptionNode::camera_info_callback, this, _1));
            image_info_sub = this->create_subscription<sensor_msgs::msg::Image>("/camera/image", 10, std::bind(&PerceptionNode::image_info_callback, this, _1));

            //PARAMETER
            this->declare_parameter<std::string>("cam_base_topic", "/camera/image");
            this->declare_parameter<double>("marker_size", 0.2); 
            this->declare_parameter<double>("target_side_px", 166.0);
            this->declare_parameter<double>("k_v", 0.005); 
            this->declare_parameter<double>("k_w", 0.005);

            cam_base_topic = this->get_parameter("cam_base_topic").as_string();
            L = this->get_parameter("marker_size").as_double();
            sd = this->get_parameter("target_side_px").as_double();
            k_v = this->get_parameter("k_v").as_double();
            k_w = this->get_parameter("k_w").as_double();

            RCLCPP_INFO(this->get_logger(),"Perception node started!");
        }

    private:

        void aruco_detect_callback(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg){

            if (msg->markers.empty() || fx == 0.0){
                return;
            }

            double z = msg->markers[0].pose.position.z;

            geometry_msgs::msg::Twist vel;
            s = fx * L / z;

            double board_error = sd - s;

            RCLCPP_INFO(this->get_logger(), "s = %.1f px | errore = %.1f px | z = %.2f m", s, board_error, z);

            if (std::abs(board_error) < 1){
                vel.linear.x = 0.0;
                vel.angular.z = 0.0;
            }else{
                vel.linear.x = k_v * board_error;
                vel.angular.z = 0.0;
            }
            intermediate_vel_pub->publish(vel);
        }

        void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg){
            fx = msg->k[0];
            cx = msg->k[2];
            fy = msg->k[4];
            cy = msg->k[5];
        }

        void image_info_callback(const sensor_msgs::msg::Image::SharedPtr msg){
           int b=0;
        }

        //PUBLISHER
        rclcpp::Publisher<robot_interfaces::msg::MarkerImage>::SharedPtr marker_img_pub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr intermediate_vel_pub;

        //SUBSCRIBERS
        rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr aruco_detection_sub;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_info_sub;

        //PARAMETERS
        std::string cam_base_topic;
        double L;
        double sd;
        double k_v, k_w;

        //VARIABLES
        double fx=0.0, fy=0.0;
        double cx=0.0, cy=0.0;
        double s;

        //TIMERS
        rclcpp::TimerBase::SharedPtr timer_;

};


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PerceptionNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
