#include <memory>
#include <string>

#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>

#include "apriltag_detector/apriltag_detector.hpp"

namespace tag_detector
{

class DetectorNode : public rclcpp::Node
{
public:
  explicit DetectorNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("apriltag_detector", options)
  {
    DetectorConfig cfg;
    cfg.family = declare_parameter<std::string>("family", cfg.family);
    cfg.quad_decimate = declare_parameter<double>("quad_decimate", cfg.quad_decimate);
    cfg.quad_sigma = declare_parameter<double>("quad_sigma", cfg.quad_sigma);
    cfg.nthreads = declare_parameter<int>("nthreads", cfg.nthreads);
    cfg.refine_edges = declare_parameter<bool>("refine_edges", cfg.refine_edges);
    cfg.decode_sharpening =
      declare_parameter<double>("decode_sharpening", cfg.decode_sharpening);
    cfg.min_decision_margin =
      declare_parameter<double>("min_decision_margin", cfg.min_decision_margin);

    publish_overlay_ = declare_parameter<bool>("publish_overlay", true);

    detector_ = std::make_unique<ApriltagDetector>(cfg);

    detections_pub_ =
      create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("detections", 10);

    RCLCPP_INFO(
      get_logger(), "AprilTag detector started (family=%s, decimate=%.1f)",
      cfg.family.c_str(), cfg.quad_decimate);
  }

  void setup_subscriptions()
  {
    auto qos = rmw_qos_profile_sensor_data;
    image_sub_ = image_transport::create_subscription(
      this, "image",
      std::bind(&DetectorNode::image_callback, this, std::placeholders::_1),
      "raw", qos);

    if (publish_overlay_) {
      overlay_pub_ = image_transport::create_publisher(this, "detections/overlay");
    }
  }

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN(get_logger(), "cv_bridge conversion failed: %s", e.what());
      return;
    }

    const auto detections = detector_->detect(cv_ptr->image);

    apriltag_msgs::msg::AprilTagDetectionArray array_msg;
    array_msg.header = msg->header;
    for (const auto & d : detections) {
      apriltag_msgs::msg::AprilTagDetection dm;
      dm.family = d.family;
      dm.id = d.id;
      dm.hamming = d.hamming;
      dm.decision_margin = static_cast<float>(d.decision_margin);
      dm.centre.x = d.center.x;
      dm.centre.y = d.center.y;
      for (int k = 0; k < 4; ++k) {
        dm.corners[k].x = d.corners[k].x;
        dm.corners[k].y = d.corners[k].y;
      }
      array_msg.detections.push_back(dm);
    }
    detections_pub_->publish(array_msg);

    if (publish_overlay_ && overlay_pub_.getNumSubscribers() > 0) {
      publish_overlay(msg, detections);
    }
  }

  void publish_overlay(
    const sensor_msgs::msg::Image::ConstSharedPtr & msg,
    const std::vector<Detection> & detections)
  {
    cv::Mat color;
    try {
      color = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN(get_logger(), "overlay conversion failed: %s", e.what());
      return;
    }

    for (const auto & d : detections) {
      for (int k = 0; k < 4; ++k) {
        cv::line(color, d.corners[k], d.corners[(k + 1) % 4], cv::Scalar(0, 255, 0), 2);
      }
      cv::putText(
        color, std::to_string(d.id), d.center, cv::FONT_HERSHEY_SIMPLEX, 0.7,
        cv::Scalar(0, 0, 255), 2);
    }

    auto out = cv_bridge::CvImage(msg->header, "bgr8", color).toImageMsg();
    overlay_pub_.publish(out);
  }

  std::unique_ptr<ApriltagDetector> detector_;
  bool publish_overlay_{true};

  image_transport::Subscriber image_sub_;
  image_transport::Publisher overlay_pub_;
  rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_pub_;
};

}  // namespace tag_detector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<tag_detector::DetectorNode>(options);
  node->setup_subscriptions();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
