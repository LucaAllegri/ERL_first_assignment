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
        marker_img_pub = this->create_publisher<sensor_msgs::msg::Image>("/marker_image", 10);
        size_error_pub = this->create_publisher<std_msgs::msg::Float64>("/size_error", 10);

        // SUBSCRIBERS
        aruco_detection_sub = this->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>("/aruco_detections", 10,std::bind(&PerceptionNode::aruco_detect_callback, this, _1));
        camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>("/camera/camera_info", 10,std::bind(&PerceptionNode::camera_info_callback, this, _1));
        image_info_sub = this->create_subscription<sensor_msgs::msg::Image>("/camera/image", 10,std::bind(&PerceptionNode::image_info_callback, this, _1));

        // PARAMETERS
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


        RCLCPP_INFO(this->get_logger(), "Perception node started!");
    }

private:

    // ARUCO CALLBACK
    void aruco_detect_callback(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg) {
        
        if (msg->markers.empty() || fx == 0.0) {
            return;
        }

        double z = msg->markers[0].pose.position.z;

        if (z <= 0.0) {
            return;
        }

        s = fx * L / z;

        std_msgs::msg::Float64 board_error;
        board_error.data = sd - s;

        /*if(board_error.data <= 1.0){


            robot_interfaces::msg::MarkerImage first_marker;
            first_marker.marker_id = 1;

            cv::circle(cv_ptr->image, cv::Point(cx, cy), 0.5, cv::Scalar(0, 255, 0), 3);

            cv::putText(cv_ptr->image, "id " + std::to_string(first_marker.marker_id), cv::Point(cx - 0.5, cy - 0.5 - 10), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0),2);
        
            first_marker.circle_image = *cv_ptr->toImageMsg();
            marker_img_pub->publish(first_marker);
        
        }*/

        if(board_error.data <= 1.0){


            sensor_msgs::msg::Image first_marker;
            //first_marker.marker_id = 1;

            cv::circle(cv_ptr->image, cv::Point(cx, cy), 20, cv::Scalar(0, 255, 0), 3);

            cv::putText(cv_ptr->image, "id 43", cv::Point(cx - 20, cy - 20 - 10), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0),2);
        
            first_marker = *cv_ptr->toImageMsg();
            marker_img_pub->publish(first_marker);
        
        }

        size_error_pub->publish(board_error);
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
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr marker_img_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr size_error_pub;

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
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;

    double s = 0.0;

    cv_bridge::CvImagePtr cv_ptr;

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