#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "apriltag_pose_estimator/pose_estimator.hpp"
#include "apriltag_pose_estimator/tag_map.hpp"

namespace apriltag_pose_estimator
{

class PoseEstimatorNode : public rclcpp::Node
{
public:
  explicit PoseEstimatorNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("apriltag_pose_estimator", options)
  {
    const std::string tag_map_path = declare_parameter<std::string>("tag_map", "");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    camera_optical_frame_ =
      declare_parameter<std::string>("camera_optical_frame", "");
    publish_tf_ = declare_parameter<bool>("publish_tf", false);
    publish_markers_ = declare_parameter<bool>("publish_markers", true);

    CovarianceModel cov;
    cov.base_pos_std = declare_parameter<double>("cov.base_pos_std", cov.base_pos_std);
    cov.base_rot_std = declare_parameter<double>("cov.base_rot_std", cov.base_rot_std);
    cov.pos_px_scale = declare_parameter<double>("cov.pos_px_scale", cov.pos_px_scale);
    cov.rot_px_scale = declare_parameter<double>("cov.rot_px_scale", cov.rot_px_scale);
    cov.pos_dist_scale = declare_parameter<double>("cov.pos_dist_scale", cov.pos_dist_scale);
    cov.rot_dist_scale = declare_parameter<double>("cov.rot_dist_scale", cov.rot_dist_scale);
    cov.single_tag_rot_factor =
      declare_parameter<double>("cov.single_tag_rot_factor", cov.single_tag_rot_factor);
    cov.max_reproj_rms = declare_parameter<double>("cov.max_reproj_rms", cov.max_reproj_rms);
    estimator_ = std::make_unique<PoseEstimator>(cov);

    MapBounds bounds;
    bounds.x_limit = declare_parameter<double>("bounds.x_limit", 9.7);
    bounds.y_limit = declare_parameter<double>("bounds.y_limit", 6.7);
    bounds.z_min = declare_parameter<double>("bounds.z_min", -1.0);
    bounds.z_max = declare_parameter<double>("bounds.z_max", 3.0);
    estimator_->set_bounds(bounds);

    // Reject detections whose image quad is smaller than this (too far / oblique).
    min_tag_area_px_ = declare_parameter<double>("min_tag_area_px", 300.0);

    std::string error;
    if (tag_map_path.empty() || !tag_map_.load_from_yaml(tag_map_path, error)) {
      RCLCPP_FATAL(
        get_logger(), "could not load tag map: %s",
        tag_map_path.empty() ? "parameter 'tag_map' is empty" : error.c_str());
      throw std::runtime_error("tag map load failed");
    }
    map_frame_ = tag_map_.frame_id();
    RCLCPP_INFO(
      get_logger(), "loaded %zu tags into frame '%s'",
      tag_map_.tags().size(), map_frame_.c_str());

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "localization/pose", 10);

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera_info", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        set_intrinsics(*msg);
      });

    detections_sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      "detections", rclcpp::SensorDataQoS(),
      std::bind(&PoseEstimatorNode::detections_callback, this, std::placeholders::_1));

    if (publish_markers_) {
      rclcpp::QoS marker_qos(1);
      marker_qos.transient_local();
      marker_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("tag_markers", marker_qos);
      marker_timer_ = create_wall_timer(
        std::chrono::seconds(1), std::bind(&PoseEstimatorNode::publish_markers, this));
    }
  }

private:
  void set_intrinsics(const sensor_msgs::msg::CameraInfo & msg)
  {
    CameraIntrinsics intr;
    intr.K = cv::Matx33d(
      msg.k[0], msg.k[1], msg.k[2],
      msg.k[3], msg.k[4], msg.k[5],
      msg.k[6], msg.k[7], msg.k[8]);
    if (!msg.d.empty()) {
      intr.dist = cv::Mat(1, static_cast<int>(msg.d.size()), CV_64F);
      for (size_t i = 0; i < msg.d.size(); ++i) {
        intr.dist.at<double>(0, static_cast<int>(i)) = msg.d[i];
      }
    }
    intrinsics_ = intr;
    have_intrinsics_ = true;
  }

  void detections_callback(
    const apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr & msg)
  {
    if (!have_intrinsics_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "waiting for camera_info");
      return;
    }
    if (msg->detections.empty()) {
      return;
    }

    std::vector<DetectionInput> inputs;
    inputs.reserve(msg->detections.size());
    for (const auto & d : msg->detections) {
      DetectionInput in;
      in.id = d.id;
      for (int k = 0; k < 4; ++k) {
        in.corners[k] = cv::Point2d(d.corners[k].x, d.corners[k].y);
      }
      if (quad_area(in.corners) < min_tag_area_px_) {
        continue;  // too far or too oblique to trust
      }
      inputs.push_back(in);
    }
    if (inputs.empty()) {
      return;
    }

    const auto estimate = estimator_->estimate_camera_pose(inputs, tag_map_, intrinsics_);
    if (!estimate.valid) {
      return;
    }

    const std::string cam_frame =
      camera_optical_frame_.empty() ? msg->header.frame_id : camera_optical_frame_;

    Eigen::Isometry3d T_cam_base;
    try {
      const auto tf = tf_buffer_->lookupTransform(
        cam_frame, base_frame_, tf2::TimePointZero);
      T_cam_base = tf2::transformToEigen(tf);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "no transform %s -> %s: %s", cam_frame.c_str(), base_frame_.c_str(), e.what());
      return;
    }

    const Eigen::Isometry3d T_map_base = estimate.T_map_camera * T_cam_base;

    geometry_msgs::msg::PoseWithCovarianceStamped out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = map_frame_;
    out.pose.pose = tf2::toMsg(T_map_base);
    for (int r = 0; r < 6; ++r) {
      for (int c = 0; c < 6; ++c) {
        out.pose.covariance[r * 6 + c] = estimate.covariance(r, c);
      }
    }
    pose_pub_->publish(out);

    if (publish_tf_ && tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header = out.header;
      tf_msg.child_frame_id = base_frame_;
      const Eigen::Quaterniond q(T_map_base.rotation());
      tf_msg.transform.translation.x = T_map_base.translation().x();
      tf_msg.transform.translation.y = T_map_base.translation().y();
      tf_msg.transform.translation.z = T_map_base.translation().z();
      tf_msg.transform.rotation.x = q.x();
      tf_msg.transform.rotation.y = q.y();
      tf_msg.transform.rotation.z = q.z();
      tf_msg.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(tf_msg);
    }
  }

  void publish_markers()
  {
    visualization_msgs::msg::MarkerArray array;
    for (const auto & [id, info] : tag_map_.tags()) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = map_frame_;
      m.header.stamp = now();
      m.ns = "apriltags";
      m.id = id;
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;
      const Eigen::Vector3d t = info.pose_map.translation();
      const Eigen::Quaterniond q(info.pose_map.rotation());
      m.pose.position.x = t.x();
      m.pose.position.y = t.y();
      m.pose.position.z = t.z();
      m.pose.orientation.x = q.x();
      m.pose.orientation.y = q.y();
      m.pose.orientation.z = q.z();
      m.pose.orientation.w = q.w();
      m.scale.x = info.size;
      m.scale.y = info.size;
      m.scale.z = 0.01;
      m.color.r = 0.1f;
      m.color.g = 0.1f;
      m.color.b = 0.1f;
      m.color.a = 1.0f;
      array.markers.push_back(m);
    }
    marker_pub_->publish(array);
  }

  static double quad_area(const std::array<cv::Point2d, 4> & c)
  {
    double a = 0.0;
    for (int i = 0; i < 4; ++i) {
      const auto & p = c[i];
      const auto & q = c[(i + 1) % 4];
      a += p.x * q.y - q.x * p.y;
    }
    return std::abs(a) * 0.5;
  }

  std::unique_ptr<PoseEstimator> estimator_;
  TagMap tag_map_;
  std::string map_frame_;
  std::string base_frame_;
  std::string camera_optical_frame_;
  bool publish_tf_{false};
  bool publish_markers_{true};
  double min_tag_area_px_{400.0};

  CameraIntrinsics intrinsics_;
  bool have_intrinsics_{false};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr
    detections_sub_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
};

}  // namespace apriltag_pose_estimator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<apriltag_pose_estimator::PoseEstimatorNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
