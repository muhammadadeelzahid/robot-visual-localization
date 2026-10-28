#ifndef APRILTAG_POSE_ESTIMATOR__POSE_ESTIMATOR_HPP_
#define APRILTAG_POSE_ESTIMATOR__POSE_ESTIMATOR_HPP_

#include <array>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include "apriltag_pose_estimator/tag_map.hpp"

namespace apriltag_pose_estimator
{

/// Pinhole intrinsics and distortion for the camera.
struct CameraIntrinsics
{
  cv::Matx33d K{cv::Matx33d::eye()};
  cv::Mat dist;  ///< May be empty (treated as zero distortion).
};

/// A detected tag's corner pixels, in AprilTag library order
/// (bottom-left, bottom-right, top-right, top-left).
struct DetectionInput
{
  int id{-1};
  std::array<cv::Point2d, 4> corners;
};

/// Heuristic mapping from reprojection error and range to a 6-DoF covariance.
struct CovarianceModel
{
  double base_pos_std{0.02};        ///< metres
  double base_rot_std{0.02};        ///< radians
  double pos_px_scale{0.01};        ///< metres of std per pixel of reprojection RMS
  double rot_px_scale{0.02};        ///< radians of std per pixel of reprojection RMS
  double pos_dist_scale{0.01};      ///< metres of std per metre of range
  double rot_dist_scale{0.03};      ///< radians of std per metre of range
  double single_tag_rot_factor{4.0};  ///< extra rotational std when only one tag
  double max_reproj_rms{4.0};       ///< px; solves worse than this are rejected
};

/// Pose of a single tag expressed in the camera optical frame.
struct SingleTagPose
{
  int id{-1};
  Eigen::Isometry3d T_cam_tag;   ///< maps tag-frame points into the camera optical frame
  double reproj_rms{0.0};
  bool valid{false};
};

/// Camera optical-frame pose in the map frame, with uncertainty.
struct CameraPoseEstimate
{
  bool valid{false};
  Eigen::Isometry3d T_map_camera;   ///< T_map_cameraoptical
  double reproj_rms{0.0};
  int num_tags{0};
  Eigen::Matrix<double, 6, 6> covariance{Eigen::Matrix<double, 6, 6>::Identity()};
};

/// Axis-aligned limits a valid camera pose must satisfy.
struct MapBounds
{
  double x_limit{12.0};
  double y_limit{9.0};
  double z_min{-1.0};
  double z_max{3.0};
};

/// Stateless PnP estimator. All geometry uses the camera optical frame
/// convention (+x right, +y down, +z forward).
class PoseEstimator
{
public:
  explicit PoseEstimator(const CovarianceModel & cov = CovarianceModel{});

  void set_bounds(const MapBounds & bounds) { bounds_ = bounds; }

  /// Single planar tag via SOLVEPNP_IPPE_SQUARE. `size` is the tag edge length.
  bool estimate_single_tag(
    const std::array<cv::Point2d, 4> & corners, double size,
    const CameraIntrinsics & intr, SingleTagPose & out) const;

  /// All IPPE solutions for a single tag (one or two), best reprojection first.
  std::vector<SingleTagPose> estimate_single_tag_solutions(
    const std::array<cv::Point2d, 4> & corners, double size,
    const CameraIntrinsics & intr) const;

  /// Camera pose in the map frame from one or more tag detections.
  CameraPoseEstimate estimate_camera_pose(
    const std::vector<DetectionInput> & detections, const TagMap & map,
    const CameraIntrinsics & intr) const;

  /// Object points of a tag's corners in the tag frame, AprilTag order.
  static std::array<cv::Point3d, 4> tag_object_points(double size);

private:
  Eigen::Matrix<double, 6, 6> build_covariance(
    double reproj_rms, int num_tags, double mean_range) const;

  bool within_bounds(const Eigen::Vector3d & p) const;

  CovarianceModel cov_;
  MapBounds bounds_;
};

}  // namespace apriltag_pose_estimator

#endif  // APRILTAG_POSE_ESTIMATOR__POSE_ESTIMATOR_HPP_
