#include <chrono>
#include <memory>
#include <cmath>
#include <algorithm>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_interfaces/msg/errors_msg.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/u_int16.hpp"

enum class State {
    APPROACH,
    TURN,
    SCAN,
    WAIT,
    TREASUREFOUND
};

class RobotMotion : public rclcpp::Node {

public:

    RobotMotion() : Node("motion") {

        // PUBLISHERS
        cmd_vel_pub_ = create_publisher<Twist>("/cmd_vel", 10);
        target_pub_ = create_publisher<std_msgs::msg::UInt16>(
            "/target_marker_id", rclcpp::QoS(1).transient_local().reliable());
        reached_pub_ = create_publisher<Errors>("/marker_reached", 10);

        // SUBSCRIBERS
        errors_sub_ = create_subscription<Errors>(
            "/errors_topic", 10,
            [this](Errors::ConstSharedPtr message) { errors_callback(message); });
        odom_sub_ = create_subscription<Odometry>(
            "/odom", rclcpp::SensorDataQoS(),
            [this](Odometry::ConstSharedPtr message) { odom_callback(*message); });

        // PARAMETERS
        k_v = positive_parameter("k_v", 0.002);
        k_w = positive_parameter("k_w", 0.001);
        k_a = positive_parameter("k_a", 0.3);
        distance_tolerance_ = positive_parameter("distance_tolerance_px", 1.0);
        center_tolerance_ = positive_parameter("center_tolerance_px", 5.0);
        turn_tolerance_ = positive_parameter("turn_tolerance_rad", 0.05);
        observation_timeout_ = positive_parameter("observation_timeout_s", 0.5);
        odom_timeout_ = positive_parameter("odom_timeout_s", 0.5);
        max_linear_speed_ = positive_parameter("max_linear_speed", 0.3);
        max_angular_speed_ = positive_parameter("max_angular_speed", 0.6);
        search_speed_ = positive_parameter("search_speed", 0.12);
        search_timeout_ = positive_parameter("search_timeout_s", 15.0);
        search_distance_ = positive_parameter("search_distance_m", 2.0);
        scan_speed_ = positive_parameter("scan_speed_rad_s", 0.35);
        scan_timeout_ = positive_parameter("scan_timeout_s", 25.0);
        publish_target();

        // TIMERS
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100), [this] { control_loop(); });
    }

private:

    using Clock = std::chrono::steady_clock;
    using Errors = robot_interfaces::msg::ErrorsMsg;
    using Odometry = nav_msgs::msg::Odometry;
    using Twist = geometry_msgs::msg::Twist;

    struct Pose {
        double x;
        double y;
        double yaw;
    };

    double positive_parameter(const std::string & name, const double fallback) {
        const auto value = declare_parameter(name, fallback);
        if (!std::isfinite(value) || value <= 0.0) {
            throw std::invalid_argument(name + " must be finite and positive");
        }
        return value;
    }

    static double wrap_angle(double angle) {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    bool fresh(const rclcpp::Time & stamp, const double timeout) const {
        const auto age = (now() - stamp).seconds();
        return stamp.nanoseconds() != 0 && age >= -0.05 && age <= timeout;
    }

    void errors_callback(const Errors::ConstSharedPtr & message) {
        if ((current_state == State::TURN || current_state == State::TREASUREFOUND) ||
            message->marker_id != target_id_) {
            return;
        }
        const rclcpp::Time stamp(message->header.stamp, get_clock()->get_clock_type());
        if (!fresh(stamp, observation_timeout_) ||
            (observation_stamp_ && stamp <= *observation_stamp_)) {
            return;
        }

        observation_stamp_ = stamp;
        observation_received_ = Clock::now();
        observation_ = message;
        if (message->valid) {
            current_state = State::APPROACH;
            target_seen_ = true;
            search_started_.reset();
        }
    }

    void odom_callback(const Odometry & message) {
        const rclcpp::Time stamp(message.header.stamp, get_clock()->get_clock_type());
        if (!fresh(stamp, odom_timeout_) || (odom_stamp_ && stamp <= *odom_stamp_)) {
            return;
        }
        const auto & p = message.pose.pose.position;
        const auto & q = message.pose.pose.orientation;
        const auto norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        if (!std::isfinite(norm) || norm <= 0.0 || !std::isfinite(p.x) || !std::isfinite(p.y)) {
            return;
        }
        const auto x = q.x / norm;
        const auto y = q.y / norm;
        const auto z = q.z / norm;
        const auto w = q.w / norm;
        const auto yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
        if (current_state == State::TURN && odom_stamp_) {
            turn_remaining_ -= wrap_angle(yaw - pose_.yaw);
        }
        if (current_state == State::SCAN && odom_stamp_) {
            scan_travel_ += std::abs(wrap_angle(yaw - pose_.yaw));
        }
        pose_ = {p.x, p.y, yaw};
        odom_stamp_ = stamp;
        odom_received_ = Clock::now();
    }

    void publish_target() {
        std_msgs::msg::UInt16 message;
        message.data = target_id_;
        target_pub_->publish(message);
        RCLCPP_INFO(get_logger(), "Looking for marker %u", static_cast<unsigned int>(target_id_));
    }

    void reached_marker() {
        cmd_vel_pub_->publish(Twist{});
        reached_pub_->publish(*observation_);
        if (target_id_ == 4) {
            current_state = State::TREASUREFOUND;
            RCLCPP_INFO(get_logger(), "TREASURE FOUND");
            return;
        }

        constexpr double pi = 3.14159265358979323846;
        constexpr double turns[] = {-pi / 2.0, pi, pi / 2.0};
        turn_remaining_ = turns[target_id_ - 1];
        current_state = State::TURN;
    }

    void start_scan(Clock::time_point started) {
        current_state = State::SCAN;
        scan_started_ = started;
        scan_travel_ = 0.0;
    }

    void control_loop() {
        Twist command;
        const auto received_now = Clock::now();
        if (current_state == State::TREASUREFOUND || !odom_stamp_ ||
            !fresh(*odom_stamp_, odom_timeout_) ||
            std::chrono::duration<double>(received_now - odom_received_).count() > odom_timeout_) {
            cmd_vel_pub_->publish(command);
            return;
        }

        if (current_state == State::TURN) {
            if (std::abs(turn_remaining_) <= turn_tolerance_) {
                current_state = State::APPROACH;
                ++target_id_;
                target_seen_ = false;
                observation_.reset();
                observation_stamp_.reset();
                search_origin_ = pose_;
                search_started_ = received_now;
                publish_target();
            } else {
                command.angular.z = std::clamp(
                    k_a * turn_remaining_, -max_angular_speed_, max_angular_speed_);
            }
        } else if (current_state == State::SCAN) {
            const auto elapsed = std::chrono::duration<double>(received_now - scan_started_);
            if (elapsed.count() >= scan_timeout_ || scan_travel_ >= 2.0 * std::acos(-1.0)) {
                current_state = State::WAIT;
            } else {
                command.angular.z = std::min(scan_speed_, max_angular_speed_);
            }
        } else if (current_state == State::WAIT) {
            // Stay stopped until a fresh observation of the current target arrives.
        } else if (observation_ && observation_->valid &&
                   fresh(*observation_stamp_, observation_timeout_) &&
                   std::chrono::duration<double>(
                       received_now - observation_received_).count() <= observation_timeout_) {
            const auto offset = observation_->misalignment_err.data;
            const auto distance = observation_->side_err.data;
            if (std::abs(offset) <= center_tolerance_ &&
                std::abs(distance) <= distance_tolerance_) {
                reached_marker();
                return;
            }

            command.angular.z = std::clamp(k_w * -offset,
                -max_angular_speed_, max_angular_speed_);
            if (std::abs(offset) <= center_tolerance_) {
                command.linear.x = std::clamp(k_v * distance,
                    -max_linear_speed_, max_linear_speed_);
            }
        } else if (search_started_) {
            const auto elapsed = std::chrono::duration<double>(received_now - *search_started_);
            const auto travel = std::hypot(pose_.x - search_origin_.x, pose_.y - search_origin_.y);
            if (elapsed.count() < search_timeout_ && travel < search_distance_) {
                command.linear.x = std::min(search_speed_, max_linear_speed_);
            } else {
                search_started_.reset();
                start_scan(received_now);
            }
        } else if (target_seen_) {
            start_scan(received_now);
        }
        cmd_vel_pub_->publish(command);
    }

    // PUBLISHERS
    rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr target_pub_;
    rclcpp::Publisher<Errors>::SharedPtr reached_pub_;

    // SUBSCRIBERS
    rclcpp::Subscription<Errors>::SharedPtr errors_sub_;
    rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;

    // TIMERS
    rclcpp::TimerBase::SharedPtr timer_;

    // PARAMETERS
    double k_v;
    double k_w;
    double k_a;
    double distance_tolerance_;
    double center_tolerance_;
    double turn_tolerance_;
    double observation_timeout_;
    double odom_timeout_;
    double max_linear_speed_;
    double max_angular_speed_;
    double search_speed_;
    double search_timeout_;
    double search_distance_;
    double scan_speed_;
    double scan_timeout_;


    // VARIABLES
    State current_state = State::APPROACH;
    std::uint16_t target_id_ = 1;
    Pose pose_{};
    Pose search_origin_{};
    bool target_seen_ = false;
    Clock::time_point scan_started_{};
    double scan_travel_ = 0.0;
    double turn_remaining_ = 0.0;
    std::optional<rclcpp::Time> odom_stamp_;
    std::optional<rclcpp::Time> observation_stamp_;
    std::optional<Clock::time_point> search_started_;
    Clock::time_point odom_received_{};
    Clock::time_point observation_received_{};
    Errors::ConstSharedPtr observation_;

};

int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<RobotMotion>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
