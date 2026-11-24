#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_interfaces/msg/marker_image.hpp"
#include "robot_interfaces/msg/errors_msg.hpp"
#include "aruco_opencv_msgs/msg/aruco_detection.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "std_msgs/msg/float64.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class PerceptionNode : public rclcpp::Node {

public:

    PerceptionNode() : Node("perception_node") {

        // PARAMETERS DECLARATION
        this->declare_parameter<std::string>("marker_image_topic", "/marker_image");
        this->declare_parameter<std::string>("circle_image_topic", "/circle_pub_topic");
        this->declare_parameter<std::string>("errors_topic", "/errors_topic");
        this->declare_parameter<std::string>("aruco_detections_topic", "/aruco_detections");
        this->declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
        this->declare_parameter<std::string>("camera_image_topic", "/camera/image");

        this->declare_parameter<double>("marker_side", 0.2);
        this->declare_parameter<double>("target_side_px", 166.0);

        // PARAMETERS
        std::string marker_img_topic = this->get_parameter("marker_image_topic").as_string();
        std::string circle_img_topic = this->get_parameter("circle_image_topic").as_string();
        std::string err_topic        = this->get_parameter("errors_topic").as_string();
        std::string aruco_topic      = this->get_parameter("aruco_detections_topic").as_string();
        std::string cam_info_topic   = this->get_parameter("camera_info_topic").as_string();
        std::string cam_img_topic    = this->get_parameter("camera_image_topic").as_string();

        L  = this->get_parameter("marker_side").as_double();
        sd = this->get_parameter("target_side_px").as_double();

        // PUBLISHERS
        marker_img_pub = this->create_publisher<robot_interfaces::msg::MarkerImage>(marker_img_topic, 10);
        circle_pub     = this->create_publisher<sensor_msgs::msg::Image>(circle_img_topic, 10);
        errors_pub     = this->create_publisher<robot_interfaces::msg::ErrorsMsg>(err_topic, 10);

        // SUBSCRIBERS
        aruco_detection_sub = this->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(aruco_topic, 10, std::bind(&PerceptionNode::aruco_detect_callback, this, _1));
        camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(cam_info_topic, 10, std::bind(&PerceptionNode::camera_info_callback, this, _1));
        image_info_sub = this->create_subscription<sensor_msgs::msg::Image>(cam_img_topic, 10, std::bind(&PerceptionNode::image_info_callback, this, _1));
        //image_info_sub = this->create_subscription<sensor_msgs::msg::Image>("/", 10, std::bind(&PerceptionNode::image_info_callback, this, _1));


        // VARIABLES 
        id_to_reach = 1;
        found = false;
        flag = true;
        fx = 0.0;
        fy = 0.0;
        cx = 0.0;
        cy = 0.0;
        s = 0.0;

        RCLCPP_INFO(this->get_logger(), "Perception node started!");
        RCLCPP_INFO(this->get_logger(), "%s", cam_img_topic.c_str());

    }

private:

    void aruco_detect_callback(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg) {
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Ciao sono una callbakc che funziona");

        if (msg->markers.empty() || fx == 0.0) {
            return;
        }

        int marker_id = 0;
        found = false;
        double x = 0;
        double y = 0;
        double z = 0;

        for(size_t i = 0; i < msg->markers.size(); i++){
            if(id_to_reach == msg->markers[i].marker_id){
                x = msg->markers[i].pose.position.x;
                y = msg->markers[i].pose.position.y;
                z = msg->markers[i].pose.position.z;
                marker_id = msg->markers[i].marker_id;
                found = true;
            }
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "marker_id detected: %d", marker_id);
        
        if(found){

            //RCLCPP_INFO(this->get_logger(), "B");
            if (z <= 0.0) {
                return;
            }

            s = fx * L / z;

            sensor_msgs::msg::Image circle_image;

            int u = static_cast<int>(fx * x / z + cx);   // centro del marker, in pixel
            int v = static_cast<int>(fy * y / z + cy);
            int r = static_cast<int>(std::ceil(s * 0.7071));

            errors.side_err.data = sd - s;
            errors.misalignment_err.data = u - cx;
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "side %.1f", errors.side_err.data);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "mis %.1f", errors.misalignment_err.data);

            if(errors.side_err.data <= 1.0 && flag == true){

                cv::circle(cv_ptr->image, cv::Point(u, v), r, cv::Scalar(0, 255, 0), 3);

                cv::putText(cv_ptr->image, "id " + std::to_string(marker_id), cv::Point(u - r, v - r - 10), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0),2);
            
                circle_image = *cv_ptr->toImageMsg();
                circle_pub->publish(circle_image);

                marker_msg.id = marker_id;
                marker_msg.circle_image = circle_image;
                marker_img_pub->publish(marker_msg);
                id_to_reach++;
                flag = false;


            } else if(errors.side_err.data > 1.0 && flag == false){
                flag = true;
            }

        }else{
            errors.side_err.data = 10;
            errors.misalignment_err.data = 0;

     
        }
        errors_pub->publish(errors);
 
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
    rclcpp::Publisher<robot_interfaces::msg::ErrorsMsg>::SharedPtr errors_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr circle_pub;

    // SUBSCRIBERS
    rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr aruco_detection_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_info_sub;

    // PARAMETERS
    double sd;
    double L;

    // VARIABLES
    robot_interfaces::msg::MarkerImage marker_msg;
    robot_interfaces::msg::ErrorsMsg errors;
    cv_bridge::CvImagePtr cv_ptr;
    int id_to_reach;
    bool flag;
    bool found;
    double fx;
    double fy;
    double cx;
    double cy;
    double s;
};


int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<PerceptionNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}