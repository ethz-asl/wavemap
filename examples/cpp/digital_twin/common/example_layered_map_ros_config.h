#ifndef WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_EXAMPLE_LAYERED_MAP_ROS_CONFIG_H_
#define WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_EXAMPLE_LAYERED_MAP_ROS_CONFIG_H_

#include <tuple>

#include <wavemap_ros_conversions/layered_map_msg_conversions.h>

#include <wavemap_ros_conversions/discrete_layer_marker_conversions.h>
#include "example_layered_map_config.h"

namespace wavemap::convert {
template <>
struct DiscreteLayerBundleRosTraits<::ExampleDiscreteLayers> {
  static auto layers(ExampleDiscreteLayers& layers) {
    return std::make_tuple(
        namedDiscreteLayerForRos("semantic", layers.semantic),
        namedDiscreteLayerForRos("changed", layers.changed));
  }

  static auto layers(const ExampleDiscreteLayers& layers) {
    return std::make_tuple(
        namedDiscreteLayerForRos("semantic", layers.semantic),
        namedDiscreteLayerForRos("changed", layers.changed));
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
        namedDiscreteMarkerLayer("semantic", layers.semantic,
                                 ExampleSemanticColorPolicy{}),
        namedDiscreteMarkerLayer("changed", layers.changed,
                                 ExampleChangedColorPolicy{}));
  }
};
}  // namespace layered_map_viz

#endif  // WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_EXAMPLE_LAYERED_MAP_ROS_CONFIG_H_
