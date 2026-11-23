#include <memory>
#include <cmath>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "robot_interfaces/msg/errors_msg.hpp"
#include "aruco_opencv_msgs/msg/aruco_detection.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/imgproc.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/u_int16.hpp"

class PerceptionNode : public rclcpp::Node {

public:

    PerceptionNode() : Node("perception_node") {

        // PUBLISHERS
        marker_img_pub = image_transport::create_publisher(this, "/marker_image");
        circle_pub = create_publisher<Image>("/circle_pub_topic", 10);
        errors_pub = create_publisher<Errors>("/errors_topic", 10);


        // SUBSCRIBERS
        camera_info_sub = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera_info", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
                camera_info_ = message;
            });
        image_info_sub = create_subscription<Image>(
            "/camera/image", rclcpp::SensorDataQoS(),
            [this](Image::ConstSharedPtr image) {
                images_.push_back(image);
                if (images_.size() > cache_size) {
                    images_.pop_front();
                }
                if (pending_ && same_frame(pending_->header, image->header)) {
                    auto detection = pending_;
                    pending_.reset();
                    measure(*detection, image);
                }
            });
        aruco_detection_sub = create_subscription<Detection>(
            "/aruco_detections", 10, [this](Detection::ConstSharedPtr detection) {
                const auto image = std::find_if(images_.rbegin(), images_.rend(),
                    [&detection](const auto & frame) {
                        return same_frame(detection->header, frame->header);
                    });
                if (image == images_.rend()) {
                    pending_ = detection;
                    return;
                }
                pending_.reset();
                measure(*detection, *image);
            });
        target_sub_ = create_subscription<std_msgs::msg::UInt16>(
            "/target_marker_id", rclcpp::QoS(1).transient_local().reliable(),
            [this](std_msgs::msg::UInt16::ConstSharedPtr message) {
                target_id_ = message->data;
                pending_.reset();
            });
        reached_sub_ = create_subscription<Errors>(
            "/marker_reached", 10,
            [this](Errors::ConstSharedPtr message) { publish_image(*message); });
        // PARAMETERS
        L = declare_parameter("marker_side", 0.2);
        sd = declare_parameter("target_side_px", 166.0);
        reference_fx_ = declare_parameter("reference_fx_px", 0.0);
        if (!std::isfinite(reference_fx_) || reference_fx_ < 0.0) {
            throw std::invalid_argument("reference_fx_px must be finite and non-negative");
        }

    }

private:

    using Detection = aruco_opencv_msgs::msg::ArucoDetection;
    using Errors = robot_interfaces::msg::ErrorsMsg;
    using Image = sensor_msgs::msg::Image;

    struct Observation {
        Errors errors;
        Image::ConstSharedPtr image;
        double u;
        double v;
        double side;
    };

    static bool same_frame(const std_msgs::msg::Header & a, const std_msgs::msg::Header & b) {
        return a.stamp == b.stamp && a.frame_id == b.frame_id;
    }

    void measure(const Detection & detection, const Image::ConstSharedPtr & image) {
        if (target_id_ == 0) {
            return;
        }

        Errors errors;
        errors.header = detection.header;
        errors.marker_id = target_id_;
        const auto marker = std::find_if(detection.markers.begin(), detection.markers.end(),
            [this](const auto & candidate) { return candidate.marker_id == target_id_; });
        if (marker == detection.markers.end() || !camera_info_) {
            errors_pub->publish(errors);
            return;
        }

        const auto & p = marker->pose.position;
        const auto & k = camera_info_->k;
        if (!(p.z > 0.0 && k[0] > 0.0 && k[4] > 0.0)) {
            errors_pub->publish(errors);
            return;
        }

        const auto u = k[0] * p.x / p.z + k[2];
        const auto v = k[4] * p.y / p.z + k[5];
        const auto side = k[0] * L / p.z;
        if (!(std::isfinite(u) && std::isfinite(v) && std::isfinite(side) && side > 0.0 &&
              u >= 0.0 && u < image->width && v >= 0.0 && v < image->height &&
              side <= 2.0 * std::max(image->width, image->height))) {
            errors_pub->publish(errors);
            return;
        }

        const auto target_side = reference_fx_ > 0.0 ? sd * k[0] / reference_fx_ : sd;
        errors.side_err.data = target_side - side;
        errors.misalignment_err.data = u - k[2];
        errors.valid = std::isfinite(errors.side_err.data) &&
            std::isfinite(errors.misalignment_err.data);
        if (errors.valid) {
            observations_.push_back({errors, image, u, v, side});
            if (observations_.size() > cache_size) {
                observations_.pop_front();
            }
        }
        errors_pub->publish(errors);
    }

    void publish_image(const Errors & reached) {
        const auto found = std::find_if(observations_.begin(), observations_.end(),
            [&reached](const auto & observation) {
                return observation.errors.marker_id == reached.marker_id &&
                    same_frame(observation.errors.header, reached.header);
            });
        if (found == observations_.end()) {
            RCLCPP_WARN(get_logger(), "Frame for reached marker %u is unavailable",
                static_cast<unsigned int>(reached.marker_id));
            return;
        }

        try {
            auto annotated = cv_bridge::toCvCopy(found->image, "bgr8");
            const cv::Point center(cvRound(found->u), cvRound(found->v));
            const int radius = static_cast<int>(std::ceil(found->side * std::sqrt(0.5)));
            const cv::Scalar color(0, 255, 0);
            cv::circle(annotated->image, center, radius, color, 3);
            cv::putText(annotated->image, "id " + std::to_string(reached.marker_id),
                center - cv::Point(radius, radius + 10), cv::FONT_HERSHEY_SIMPLEX,
                1.0, color, 2);

            const auto message = annotated->toImageMsg();
            marker_img_pub.publish(*message);
            circle_pub->publish(*message);
        } catch (const cv_bridge::Exception & exception) {
            RCLCPP_WARN(get_logger(), "Could not convert marker image: %s", exception.what());
        }
        observations_.erase(found);
    }

    static constexpr std::size_t cache_size = 24;
    // PUBLISHERS
    image_transport::Publisher marker_img_pub;
    rclcpp::Publisher<Image>::SharedPtr circle_pub;
    rclcpp::Publisher<Errors>::SharedPtr errors_pub;

    // SUBSCRIBERS
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;
    rclcpp::Subscription<Image>::SharedPtr image_info_sub;
    rclcpp::Subscription<Detection>::SharedPtr aruco_detection_sub;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr target_sub_;
    rclcpp::Subscription<Errors>::SharedPtr reached_sub_;

    // PARAMETERS
    double L;
    double sd;
    double reference_fx_;

    // VARIABLES
    std::uint16_t target_id_ = 0;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
    Detection::ConstSharedPtr pending_;
    std::deque<Image::ConstSharedPtr> images_;
    std::deque<Observation> observations_;

};

int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<PerceptionNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
