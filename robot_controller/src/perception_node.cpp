#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_interfaces/msg/marker_image.hpp"
#include "aruco_opencv_msgs/msg/aruco_detection.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/msg/float64.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class PerceptionNode : public rclcpp::Node {

public:

    PerceptionNode() : Node("perception_node") {

        // PUBLISHERS
        marker_img_pub = this->create_publisher<robot_interfaces::msg::MarkerImage>("/marker_image", 10);
        circle_pub = this->create_publisher<sensor_msgs::msg::Image>("/camera/image", 10);   //circle_pub_topic
        side_error_pub = this->create_publisher<std_msgs::msg::Float64>("/side_error", 10);

        // SUBSCRIBERS
        aruco_detection_sub = this->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>("/aruco_detections", 10,std::bind(&PerceptionNode::aruco_detect_callback, this, _1));
        camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>("/camera/camera_info", 10,std::bind(&PerceptionNode::camera_info_callback, this, _1));
        image_info_sub = this->create_subscription<sensor_msgs::msg::Image>("/camera/image", 10,std::bind(&PerceptionNode::image_info_callback, this, _1));

        // PARAMETERS
        this->declare_parameter<std::string>("cam_base_topic", "/camera/image");
        this->declare_parameter<double>("marker_side", 0.2);
        this->declare_parameter<double>("target_side_px", 166.0);
        this->declare_parameter<double>("k_v", 0.005);
        this->declare_parameter<double>("k_w", 0.005);

        cam_base_topic = this->get_parameter("cam_base_topic").as_string();
        L = this->get_parameter("marker_side").as_double();
        sd = this->get_parameter("target_side_px").as_double();
        k_v = this->get_parameter("k_v").as_double();
        k_w = this->get_parameter("k_w").as_double();


        RCLCPP_INFO(this->get_logger(), "Perception node started!");
    }

private:


    void aruco_detect_callback(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg) {
        
        if (msg->markers.empty() || fx == 0.0) {
            return;
        }

        int marker_id = msg->markers[0].marker_id;

        double x = msg->markers[0].pose.position.x;
        double y = msg->markers[0].pose.position.y;
        double z = msg->markers[0].pose.position.z;

        if (z <= 0.0) {
            return;
        }

        s = fx * L / z;

        side_error.data = sd - s;

        int u = static_cast<int>(fx * x / z + cx);   // centro del marker, in pixel
        int v = static_cast<int>(fy * y / z + cy);
        int r = static_cast<int>(std::ceil(s * 0.7071)); 

        if(side_error.data <= 1.0){

            cv::circle(cv_ptr->image, cv::Point(u, v), r, cv::Scalar(0, 255, 0), 3);

            cv::putText(cv_ptr->image, "id " + std::to_string(marker_id), cv::Point(u - r, v - r - 10), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0),2);
        
            circle_image = *cv_ptr->toImageMsg();
            circle_pub->publish(circle_image);

            marker_msg.id = marker_id;
            marker_msg.circle_image = circle_image;
            marker_img_pub->publish(marker_msg);
        
        }

        side_error_pub->publish(side_error);
    }

    // CAMERA INFO CALLBACK
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        fx = msg->k[0];
        cx = msg->k[2];
        fy = msg->k[4];
        cy = msg->k[5];
    }

    // IMAGE CALLBACK
    void image_info_callback(const sensor_msgs::msg::Image::SharedPtr img_msg) {
        cv_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
    }

    // PUBLISHERS
    rclcpp::Publisher<robot_interfaces::msg::MarkerImage>::SharedPtr marker_img_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr circle_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr side_error_pub;

    // SUBSCRIBERS
    rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr aruco_detection_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_info_sub;

    // PARAMETERS
    std::string cam_base_topic;
    double L;
    double sd;
    double k_v;
    double k_w;

    // VARIABLES
    robot_interfaces::msg::MarkerImage marker_msg;
    sensor_msgs::msg::Image circle_image;
    std_msgs::msg::Float64 side_error;
    cv_bridge::CvImagePtr cv_ptr;
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    double s = 0.0;

    // TIMERS
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<PerceptionNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}