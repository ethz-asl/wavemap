#ifndef WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_FIELD_LAYERED_MAP_ROS_CONFIG_H_
#define WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_FIELD_LAYERED_MAP_ROS_CONFIG_H_

#include <string>
#include <tuple>

#include <ros/console.h>
#include <ros/node_handle.h>
#include <wavemap_ros_conversions/descriptor_layered_map_conversions.h>
#include <wavemap_ros_conversions/discrete_layer_marker_conversions.h>


#include "field_layered_map_config.h"

namespace wavemap::examples::field_map::ros_config {
namespace detail {

template <typename ValueT>
void readParam(const ros::NodeHandle& nh, const std::string& key,
               ValueT& value) {
  nh.param(key, value, value);
}

}  // namespace detail

inline Config loadConfig(const ros::NodeHandle& nh_private,
                         const std::string& ns = "layered_map") {
  Config config = makeDefaultConfig();

  detail::readParam(nh_private, ns + "/continuous_map/min_cell_width/meters",
                    config.continuous_map.min_cell_width);
  detail::readParam(nh_private, ns + "/continuous_map/min_log_odds",
                    config.continuous_map.min_log_odds);
  detail::readParam(nh_private, ns + "/continuous_map/max_log_odds",
                    config.continuous_map.max_log_odds);
  detail::readParam(nh_private, ns + "/continuous_map/tree_height",
                    config.continuous_map.tree_height);
  detail::readParam(
      nh_private, ns + "/continuous_map/only_prune_blocks_if_unused_for/seconds",
      config.continuous_map.only_prune_blocks_if_unused_for);

  detail::readParam(nh_private, ns + "/continuous_pruning/combined_threshold",
                    config.continuous_pruning.combined_threshold);
  config.continuous_pruning.data.combined_threshold =
      config.continuous_pruning.combined_threshold;

  detail::readParam(nh_private, ns + "/continuous_pruning/occupancy/scale",
                    config.continuous_pruning.occupancy.scale);
  detail::readParam(nh_private, ns + "/continuous_pruning/occupancy/weight",
                    config.continuous_pruning.occupancy.weight);
  auto& reflectivity_pruning =
      config.continuous_pruning.data.get<ReflectivityLayer>();
  detail::readParam(nh_private, ns + "/continuous_pruning/reflectivity/scale",
                    reflectivity_pruning.scale);
  detail::readParam(nh_private, ns + "/continuous_pruning/reflectivity/weight",
                    reflectivity_pruning.weight);

  detail::readParam(nh_private, ns + "/discrete_compression/block_height",
                    config.discrete_compression.block_height);
  return config;
}

using VoxelRosConverter =
    convert::DescriptorContinuousRosConverter<Voxel>;

}  // namespace wavemap::examples::field_map::ros_config

namespace wavemap::convert {

template <>
struct DiscreteLayerBundleRosTraits<
    examples::field_map::DiscreteLayers> {
  static auto layers(examples::field_map::DiscreteLayers& layers) {
    return descriptorNamedDiscreteLayersForRos(layers);
  }

  static auto layers(const examples::field_map::DiscreteLayers& layers) {
    return descriptorNamedDiscreteLayersForRos(layers);
  }
};

}  // namespace wavemap::convert

namespace layered_map_viz {

struct FieldClassColorPolicy {
  bool operator()(int value, std_msgs::ColorRGBA& color_msg) const {
    using ClassLabel = wavemap::examples::field_map::ClassLabel;
    switch (static_cast<ClassLabel>(value)) {
      case ClassLabel::kGround:
        color_msg = color(0.55f, 0.32f, 0.12f, 0.9f);
        return true;
      case ClassLabel::kObstacle:
        color_msg = color(0.85f, 0.15f, 0.1f, 0.9f);
        return true;
      case ClassLabel::kUnknown:
      default:
        color_msg = color(0.5f, 0.5f, 0.5f, 0.5f);
        return true;
    }
  }
};

template <>
struct DiscreteLayerMarkerTraits<
    wavemap::examples::field_map::DiscreteLayers> {
  static auto layers(
      const wavemap::examples::field_map::DiscreteLayers& layers) {
    return std::make_tuple(namedDiscreteMarkerLayer(
        std::string(wavemap::layered::LayerTraits<wavemap::examples::field_map::ClassLayer>::name), layers.get<wavemap::examples::field_map::ClassLayer>(),
        FieldClassColorPolicy{}));
  }
};

}  // namespace layered_map_viz

#endif  // WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_FIELD_LAYERED_MAP_ROS_CONFIG_H_
