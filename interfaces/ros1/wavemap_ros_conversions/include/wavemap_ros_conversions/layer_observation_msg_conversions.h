#ifndef WAVEMAP_ROS_CONVERSIONS_LAYER_OBSERVATION_MSG_CONVERSIONS_H_
#define WAVEMAP_ROS_CONVERSIONS_LAYER_OBSERVATION_MSG_CONVERSIONS_H_

#include <string>
#include <vector>

#include <std_msgs/Header.h>
#include <wavemap/layered/layer_observation.h>

namespace wavemap::convert {

template <typename ValueT>
struct RosLayerObservationBatch {
  std_msgs::Header header;
  std::vector<layered::LayerObservation<ValueT>> observations;
  size_t rejected = 0u;
  std::string error;

  explicit operator bool() const { return error.empty(); }
};

}  // namespace wavemap::convert

#endif  // WAVEMAP_ROS_CONVERSIONS_LAYER_OBSERVATION_MSG_CONVERSIONS_H_
