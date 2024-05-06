#ifndef WAVEMAP_ROS_UTILS_POINTCLOUD_UNDISTORTION_POINTCLOUD_UNDISTORTER_H_
#define WAVEMAP_ROS_UTILS_POINTCLOUD_UNDISTORTION_POINTCLOUD_UNDISTORTER_H_

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <wavemap/core/common.h>
#include <wavemap/core/data_structure/pointcloud.h>

#include "wavemap_ros/utils/pointcloud_undistortion/stamped_pointcloud.h"
#include "wavemap_ros/utils/tf_transformer.h"

namespace wavemap {
class PointcloudUndistorter {
 public:
  enum class Result {
    kStartTimeNotInTfBuffer,
    kEndTimeNotInTfBuffer,
    kIntermediateTimeNotInTfBuffer,
    kSuccess
  };

  explicit PointcloudUndistorter(std::shared_ptr<TfTransformer> transformer,
                                 int num_interpolation_intervals_per_cloud)
      : transformer_(std::move(transformer)),
        num_interpolation_intervals_per_cloud_(
            num_interpolation_intervals_per_cloud) {}

  Result undistortPointcloud(StampedPointcloud& stamped_pointcloud,
                             PosedPointcloud<>& undistorted_pointcloud,
                             const std::string& fixed_frame);

 private:
  std::shared_ptr<TfTransformer> transformer_;
  const int num_interpolation_intervals_per_cloud_;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_UTILS_POINTCLOUD_UNDISTORTION_POINTCLOUD_UNDISTORTER_H_
