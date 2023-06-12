#ifndef WAVEMAP_ROS_INPUT_HANDLER_POINTCLOUD_UNDISTORTER_H_
#define WAVEMAP_ROS_INPUT_HANDLER_POINTCLOUD_UNDISTORTER_H_

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <wavemap/common.h>
#include <wavemap/data_structure/pointcloud.h>

#include "wavemap_ros/tf_transformer.h"

namespace wavemap {
struct StampedPoint {
  Point3D point;
  uint32_t time_offset;

  StampedPoint(FloatingPoint x, FloatingPoint y, FloatingPoint z,
               uint64_t time_offset)
      : point(x, y, z), time_offset(time_offset) {}
};

struct StampedPointcloud {
  std::string world_frame;
  std::string sensor_frame;
  uint64_t time_base;
  std::vector<StampedPoint> points;

  StampedPointcloud(uint64_t time_base, std::string world_frame,
                    std::string sensor_frame, size_t expected_num_points = 0)
      : world_frame(std::move(world_frame)),
        sensor_frame(std::move(sensor_frame)),
        time_base(time_base) {
    points.reserve(expected_num_points);
  }

  template <typename... Args>
  auto emplace(Args... args) {
    return points.emplace_back(std::forward<Args>(args)...);
  }

  void sort() {
    std::sort(points.begin(), points.end(),
              [](const auto& lhs, const auto& rhs) {
                return lhs.time_offset < rhs.time_offset;
              });
  }

  uint64_t getStartTime() const {
    return time_base + points.front().time_offset;
  }
  uint64_t getEndTime() const { return time_base + points.back().time_offset; }
};

class PointcloudUndistorter {
 public:
  enum class Result {
    kStartTimeNotInTfBuffer,
    kEndTimeNotInTfBuffer,
    kIntermediateTimeNotInTfBuffer,
    kSuccess
  };

  explicit PointcloudUndistorter(std::shared_ptr<TfTransformer> transformer)
      : transformer_(std::move(transformer)) {}

  Result undistortPointcloud(StampedPointcloud& stamped_pointcloud,
                             PosedPointcloud<>& undistorted_pointcloud);

 private:
  std::shared_ptr<TfTransformer> transformer_;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_INPUT_HANDLER_POINTCLOUD_UNDISTORTER_H_
