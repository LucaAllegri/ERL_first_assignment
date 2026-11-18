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

using std::placeholders::_1;
using namespace std::chrono_literals;

enum class State {
    APPROACH,        // Visual servoing: si allinea e si ferma a 166 px
    TURN,            // Rotazione ad anello chiuso (90° dx, 180°, 90° sx)
    TREASUREFOUND   // Marker 4 raggiunto: stop definitivo
};

class RobotMotion : public rclcpp::Node {

public:

    RobotMotion() : Node("motion") {

        // PUBLISHERS
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // SUBSCRIBERS
        marker_img_sub = this->create_subscription<robot_interfaces::msg::MarkerImage>("/marker_image", 10, std::bind(&RobotMotion::marker_img_callback, this, _1));
        errors_sub = this->create_subscription<robot_interfaces::msg::ErrorsMsg>("/errors_topic", 10, std::bind(&RobotMotion::errors_callback, this, _1));
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&RobotMotion::odom_callback, this, _1));

        // TIMERS
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&RobotMotion::control_loop, this));

        // PARAMETERS
        this->declare_parameter<double>("k_v", 0.002);
        this->declare_parameter<double>("k_w", 0.001);
        this->declare_parameter<double>("k_a", 0.3);

        k_v = this->get_parameter("k_v").as_double();
        k_w = this->get_parameter("k_w").as_double();
        k_a = this->get_parameter("k_a").as_double();

        // VARIABLES
        side_error.data = 200.0;  // random value for the start

        RCLCPP_INFO(this->get_logger(), "Robot motion node started!");
    }

private:

    void control_loop(){

        geometry_msgs::msg::Twist vel;

        switch (current_state) {

            case State::APPROACH:
                
                if (std::abs(side_error.data) > 1.0){

                    vel.linear.x = k_v * side_error.data;
                    vel.angular.z = -k_w * misalignment_error.data;
                    
                    move_robot(vel);

                    RCLCPP_INFO(this->get_logger(), "side_error %.1f", side_error.data);
                }else{
                    
                    vel.linear.x = 0.0;
                    vel.angular.z = 0.0;
                    move_robot(vel);
                    RCLCPP_INFO(this->get_logger(), "robot stops");

                    if(marker_id == 4){
                        current_state = State::TREASUREFOUND;
                    }else{
                        current_state = State::TURN;
                    }
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

                RCLCPP_INFO(this->get_logger(), "robot is turning");
                
                move_robot(vel);

                if( abs(ang_error) < 0.05){
                    current_state = State::APPROACH;
                }

                break;
            }

            case State::TREASUREFOUND:
                RCLCPP_INFO(this->get_logger(), "TREASURE FOUND!!!!!!!!");
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
        RCLCPP_INFO(this->get_logger(), "id = %d", marker_id);
    }

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

        RCLCPP_INFO(this->get_logger(), "yaw %.1f", yaw);

        float err = atan2(sin(target - yaw), cos(target-yaw));

        RCLCPP_INFO(this->get_logger(), "error: %.1f", err);
        
        return err;
    }

    // PUBLISHERS
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // SUBSCRIBERS
    rclcpp::Subscription<robot_interfaces::msg::MarkerImage>::SharedPtr marker_img_sub;
    rclcpp::Subscription<robot_interfaces::msg::ErrorsMsg>::SharedPtr errors_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

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
    int marker_id;
};


int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<RobotMotion>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}