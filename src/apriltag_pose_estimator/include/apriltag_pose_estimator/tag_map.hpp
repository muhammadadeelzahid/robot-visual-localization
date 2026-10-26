#ifndef APRILTAG_POSE_ESTIMATOR__TAG_MAP_HPP_
#define APRILTAG_POSE_ESTIMATOR__TAG_MAP_HPP_

#include <map>
#include <optional>
#include <string>

#include <Eigen/Geometry>

namespace apriltag_pose_estimator
{

/// A tag's physical size and its pose in the world/map frame.
///
/// The tag frame has its origin at the tag centre, +x to the right, +y up, and
/// +z pointing out of the printed face toward the viewer.
struct TagInfo
{
  int id{-1};
  double size{0.0};               ///< Full edge length of the black square, metres.
  Eigen::Isometry3d pose_map;     ///< T_map_tag.
};

/// Collection of known tags loaded from a YAML description.
class TagMap
{
public:
  /// Parse a tag map from YAML. Returns false and fills `error` on failure.
  bool load_from_yaml(const std::string & path, std::string & error);

  std::optional<TagInfo> get(int id) const;

  /// Insert or replace a tag. Primarily for programmatic setup and tests.
  void insert(const TagInfo & info) { tags_[info.id] = info; }
  void set_frame_id(const std::string & frame) { frame_id_ = frame; }
  void set_default_size(double size) { default_size_ = size; }

  double default_size() const { return default_size_; }
  const std::string & frame_id() const { return frame_id_; }
  const std::map<int, TagInfo> & tags() const { return tags_; }
  bool empty() const { return tags_.empty(); }

private:
  std::string frame_id_{"map"};
  double default_size_{0.16};
  std::map<int, TagInfo> tags_;
};

}  // namespace apriltag_pose_estimator

#endif  // APRILTAG_POSE_ESTIMATOR__TAG_MAP_HPP_
