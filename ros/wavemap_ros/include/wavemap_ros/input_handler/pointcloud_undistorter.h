#ifndef WAVEMAP_ROS_INPUT_HANDLER_POINTCLOUD_UNDISTORTER_H_
#define WAVEMAP_ROS_INPUT_HANDLER_POINTCLOUD_UNDISTORTER_H_

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <wavemap/common.h>
#include <wavemap/data_structure/pointcloud.h>

#include "wavemap_ros/input_handler/generic_stamped_pointcloud.h"
#include "wavemap_ros/tf_transformer.h"

namespace wavemap {
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

  Result undistortPointcloud(GenericStampedPointcloud& stamped_pointcloud,
                             PosedPointcloud<>& undistorted_pointcloud,
                             const std::string& fixed_frame);

 private:
  std::shared_ptr<TfTransformer> transformer_;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_INPUT_HANDLER_POINTCLOUD_UNDISTORTER_H_
