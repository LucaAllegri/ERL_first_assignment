#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include "robot_interfaces/msg/marker_image.hpp"
#include "robot_interfaces/msg/errors_msg.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "std_msgs/msg/int32.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

enum class State {
    APPROACH,        
    TURN,            
    TREASUREFOUND   
};

class RobotMotion : public rclcpp::Node {

public:

    RobotMotion() : Node("robot_motion") {

        // PARAMETERS DECLARATION
        this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        this->declare_parameter<std::string>("marker_image_topic", "/marker_image");
        this->declare_parameter<std::string>("errors_topic", "/errors_topic");
        this->declare_parameter<std::string>("odom_topic", "/odom");

        this->declare_parameter<double>("k_v", 0.01);
        this->declare_parameter<double>("k_w", 0.01);
        this->declare_parameter<double>("k_a", 0.3);

        // PARAMETERS
        std::string cmd_vel_topic    = this->get_parameter("cmd_vel_topic").as_string();
        std::string marker_img_topic = this->get_parameter("marker_image_topic").as_string();
        std::string err_topic        = this->get_parameter("errors_topic").as_string();
        std::string odom_topic       = this->get_parameter("odom_topic").as_string();

        k_v = this->get_parameter("k_v").as_double();
        k_w = this->get_parameter("k_w").as_double();
        k_a = this->get_parameter("k_a").as_double();

        // PUBLISHERS
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

        // SUBSCRIBERS
        marker_img_sub = this->create_subscription<robot_interfaces::msg::MarkerImage>(marker_img_topic, 10, std::bind(&RobotMotion::marker_img_callback, this, _1));
        errors_sub = this->create_subscription<robot_interfaces::msg::ErrorsMsg>(err_topic, 10, std::bind(&RobotMotion::errors_callback, this, _1));
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 10, std::bind(&RobotMotion::odom_callback, this, _1));
        //DELETE COMMENT FOR REAL LIFE ROBOT
        //id_sub = this->create_subscription<std_msgs::msg::Int32>("/id_topic", 10, std::bind(&RobotMotion::id_callback, this, _1));

        timer_ = this->create_timer(100ms, std::bind(&RobotMotion::control_loop, this));

        // VARIABLES
        side_error.data = 200.0;  // random value for the start

        RCLCPP_INFO(this->get_logger(), "%s", cmd_vel_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "%s", odom_topic.c_str());

    }

private:

    void control_loop(){

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "id = %d", marker_id);

        geometry_msgs::msg::Twist vel;

        switch (current_state) {

            case State::APPROACH:
                
                if (side_error.data > 1.0){

                    vel.linear.x = k_v * side_error.data;
                    vel.angular.z = -k_w * misalignment_error.data;
                    
                    move_robot(vel);

                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10, "side_error %.1f", side_error.data);

                }else if(std::abs(side_error.data)< 1){
                    
                    vel.linear.x = 0.0;
                    vel.angular.z = 0.0;
                    move_robot(vel);
                    
                    RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(), 1000, "robot stops");

                } else{

                    vel.linear.x = 0.2;
                    vel.angular.z = 0.0;
                    move_robot(vel);
                    RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(), 1000, "robot searching");

                }

                initial_yaw = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));


                break;

            case State::TURN:{

                if (!marker_id){
                    break;
                }

                float ang_error = angular_error(marker_id);

                vel.linear.x = 0.0;     
                vel.angular.z = k_a * ang_error;

                
                move_robot(vel);

                if( abs(ang_error) < 0.05){
                    current_state = State::APPROACH;
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "STATE: TURNING->APPROACH");

                }

                break;
            }

            case State::TREASUREFOUND:

                vel.linear.x = 0.0;     
                vel.angular.z = 0.0;
                move_robot(vel);

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TREASURE FOUND!!!!!!!!");
                break;
        }
    }

    // side ERROR CALLBACK
    void errors_callback(const robot_interfaces::msg::ErrorsMsg::SharedPtr msg) {

        side_error = msg->side_err;
        misalignment_error = msg->misalignment_err;

    }

    void move_robot(const geometry_msgs::msg::Twist vel){

        cmd_vel_pub_->publish(vel);

    }

    void marker_img_callback(const robot_interfaces::msg::MarkerImage::SharedPtr msg) {
        marker_id = msg->id;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "id = %d", marker_id);

        if(marker_id == 4){
            current_state = State::TREASUREFOUND;
        }else{
            current_state = State::TURN;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "STATE: APPROACH->TURNING");
        }
    }

    //DELETE COMMENT FOR REAL LIFE ROBOT

    // void id_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    //     marker_id = msg->data;
    //     RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "id = %d", marker_id);

    //     if(marker_id == 4){
    //         current_state = State::TREASUREFOUND;
    //     }else{
    //         current_state = State::TURN;
    //         RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "STATE: APPROACH->TURNING");
    //     }
    // }

    void odom_callback(const nav_msgs::msg::Odometry msg){

        position[0] = msg.pose.pose.position.x;
        position[1] = msg.pose.pose.position.y;
        position[2] = msg.pose.pose.position.z;

        q = msg.pose.pose.orientation;
    }

    float angular_error(int id){

        double delta;

        if(id == 1){
            delta = -M_PI/2;
        }else if(id == 2){
            delta = M_PI;
        }else if(id == 3){
            delta = M_PI/2;
        }else if(id == 4){
            delta = 2*M_PI;
        }

        double yaw = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));

        double target = delta + initial_yaw;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "yaw %.1f", yaw);

        float err = atan2(sin(target - yaw), cos(target-yaw));

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "error: %.1f", err);
        
        return err;
    }

    // PUBLISHERS
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // SUBSCRIBERS
    rclcpp::Subscription<robot_interfaces::msg::MarkerImage>::SharedPtr marker_img_sub;
    rclcpp::Subscription<robot_interfaces::msg::ErrorsMsg>::SharedPtr errors_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    //rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr id_sub;


    // PARAMETERS
    double k_v;
    double k_w;
    double k_a;

    // TIMERS
    rclcpp::TimerBase::SharedPtr timer_;

    //VARIABLES
    std::array<double, 3> position = {0.0, 0.0, 0.0};
    std_msgs::msg::Float64 misalignment_error;
    State current_state = State::APPROACH;
    std_msgs::msg::Float64 side_error;
    geometry_msgs::msg::Quaternion q;
    double initial_yaw;
    int marker_id = 0;
};


int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<RobotMotion>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}