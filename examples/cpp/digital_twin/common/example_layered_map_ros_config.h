#ifndef WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_EXAMPLE_LAYERED_MAP_ROS_CONFIG_H_
#define WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_EXAMPLE_LAYERED_MAP_ROS_CONFIG_H_

#include <string>
#include <tuple>
#include <vector>

#include <ros/console.h>
#include <ros/node_handle.h>
#include <wavemap_ros_conversions/layered_map_msg_conversions.h>

#include <wavemap_ros_conversions/discrete_layer_marker_conversions.h>
#include "example_layered_map_config.h"

namespace layered_map_config {
namespace detail {
inline bool readRgbParam(const ros::NodeHandle& nh, const std::string& key,
                         Rgb& value) {
  std::vector<double> values;
  if (!nh.getParam(key, values)) {
    return false;
  }
  if (values.size() != 3u) {
    ROS_WARN_STREAM("Ignoring layered map RGB param " << key
                    << " because it must contain exactly 3 values.");
    return false;
  }
  value = rgb(static_cast<float>(values[0]), static_cast<float>(values[1]),
              static_cast<float>(values[2]));
  return true;
}

template <typename ValueT>
void readParam(const ros::NodeHandle& nh, const std::string& key,
               ValueT& value) {
  nh.param(key, value, value);
}
}  // namespace detail

inline ExampleLayeredMapConfig makeDefaultExampleLayeredMapConfig() {
  ExampleLayeredMapConfig config;
  config.continuous_map.min_cell_width = 0.1f;
  config.continuous_map.min_log_odds = -2.f;
  config.continuous_map.max_log_odds = 4.f;
  config.continuous_map.tree_height = 3;
  config.continuous_map.only_prune_blocks_if_unused_for = 5.f;

  config.continuous_threshold.data.rgb_min = rgb(0.f, 0.f, 0.f);
  config.continuous_threshold.data.rgb_max = rgb(1.f, 1.f, 1.f);
  config.continuous_threshold.data.traversability_min = 0.f;
  config.continuous_threshold.data.traversability_max = 1.f;

  config.continuous_pruning.occupancy.scale = 1e-3f;
  config.continuous_pruning.occupancy.weight = 1.f;
  config.continuous_pruning.data.rgb.scale = 1e-3f;
  config.continuous_pruning.data.rgb.weight = 1.f;
  config.continuous_pruning.data.traversability.scale = 1e-3f;
  config.continuous_pruning.data.traversability.weight = 1.f;
  config.continuous_pruning.combined_threshold = 1.f;
  config.continuous_pruning.data.combined_threshold = 1.f;

  config.discrete_compression.block_height = 1;
  return config;
}

inline ExampleLayeredMapConfig loadExampleLayeredMapConfigFromRosParams(
    const ros::NodeHandle& nh_private,
    const std::string& ns = "layered_map") {
  ExampleLayeredMapConfig config = makeDefaultExampleLayeredMapConfig();

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

  detail::readRgbParam(nh_private, ns + "/continuous_threshold/color/min",
                       config.continuous_threshold.data.rgb_min);
  detail::readRgbParam(nh_private, ns + "/continuous_threshold/color/max",
                       config.continuous_threshold.data.rgb_max);
  detail::readParam(nh_private, ns + "/continuous_threshold/traversability/min",
                    config.continuous_threshold.data.traversability_min);
  detail::readParam(nh_private, ns + "/continuous_threshold/traversability/max",
                    config.continuous_threshold.data.traversability_max);

  detail::readParam(nh_private, ns + "/continuous_pruning/combined_threshold",
                    config.continuous_pruning.combined_threshold);
  config.continuous_pruning.data.combined_threshold =
      config.continuous_pruning.combined_threshold;
  detail::readParam(nh_private, ns + "/continuous_pruning/occupancy/scale",
                    config.continuous_pruning.occupancy.scale);
  detail::readParam(nh_private, ns + "/continuous_pruning/occupancy/weight",
                    config.continuous_pruning.occupancy.weight);
  detail::readParam(nh_private, ns + "/continuous_pruning/color/scale",
                    config.continuous_pruning.data.rgb.scale);
  detail::readParam(nh_private, ns + "/continuous_pruning/color/weight",
                    config.continuous_pruning.data.rgb.weight);
  detail::readParam(nh_private, ns + "/continuous_pruning/traversability/scale",
                    config.continuous_pruning.data.traversability.scale);
  detail::readParam(nh_private, ns + "/continuous_pruning/traversability/weight",
                    config.continuous_pruning.data.traversability.weight);

  detail::readParam(nh_private, ns + "/discrete_compression/block_height",
                    config.discrete_compression.block_height);
  return config;
}
}  // namespace layered_map_config

namespace wavemap::convert {
template <>
struct DiscreteLayerBundleRosTraits<::ExampleDiscreteLayers> {
  static auto layers(ExampleDiscreteLayers& layers) {
    return std::make_tuple(
        namedDiscreteLayerForRos(kSemanticLayerName, layers.semantic),
        namedDiscreteLayerForRos(kChangedLayerName, layers.changed));
  }

  static auto layers(const ExampleDiscreteLayers& layers) {
    return std::make_tuple(
        namedDiscreteLayerForRos(kSemanticLayerName, layers.semantic),
        namedDiscreteLayerForRos(kChangedLayerName, layers.changed));
  }
};
}  // namespace wavemap::convert


namespace layered_map_viz {
struct ExampleSemanticColorPolicy {
  bool operator()(int label, std_msgs::ColorRGBA& color_msg) const {
    switch (label) {
      case 1:
        color_msg = color(0.1f, 0.6f, 1.f, 0.85f);
        return true;
      case 2:
        color_msg = color(1.f, 0.5f, 0.1f, 0.85f);
        return true;
      default:
        color_msg = color(0.7f, 0.7f, 0.7f, 0.6f);
        return true;
    }
  }
};

struct ExampleChangedColorPolicy {
  bool operator()(bool changed, std_msgs::ColorRGBA& color_msg) const {
    if (!changed) {
      return false;
    }
    color_msg = color(1.f, 0.05f, 0.05f, 1.f);
    return true;
  }
};

template <>
struct DiscreteLayerMarkerTraits<::ExampleDiscreteLayers> {
  static auto layers(const ExampleDiscreteLayers& layers) {
    return std::make_tuple(
        namedDiscreteMarkerLayer(kSemanticLayerName, layers.semantic,
                                 ExampleSemanticColorPolicy{}),
        namedDiscreteMarkerLayer(kChangedLayerName, layers.changed,
                                 ExampleChangedColorPolicy{}));
  }
};
}  // namespace layered_map_viz

#endif  // WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_EXAMPLE_LAYERED_MAP_ROS_CONFIG_H_
