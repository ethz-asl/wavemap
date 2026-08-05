#ifndef WAVEMAP_ROS_CONVERSIONS_DISCRETE_LAYER_MARKER_CONVERSIONS_H_
#define WAVEMAP_ROS_CONVERSIONS_DISCRETE_LAYER_MARKER_CONVERSIONS_H_

#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include <geometry_msgs/Point.h>
#include <ros/time.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <wavemap/core/common.h>

#include <wavemap/layered/discrete_layer.h>

namespace layered_map_viz {
namespace detail {
template <typename T>
inline constexpr bool kAlwaysFalse = false;

template <typename TupleT, typename FuncT, size_t... Indices>
bool forEachTupleElementImpl(TupleT&& tuple, FuncT&& func,
                             std::index_sequence<Indices...>) {
  bool ok = true;
  ((ok = ok && func(std::get<Indices>(tuple))), ...);
  return ok;
}

template <typename TupleT, typename FuncT>
bool forEachTupleElement(TupleT&& tuple, FuncT&& func) {
  constexpr size_t tuple_size =
      std::tuple_size_v<std::remove_reference_t<TupleT>>;
  return forEachTupleElementImpl(std::forward<TupleT>(tuple),
                                 std::forward<FuncT>(func),
                                 std::make_index_sequence<tuple_size>{});
}
}  // namespace detail

inline std_msgs::ColorRGBA color(float r, float g, float b, float a = 1.f) {
  std_msgs::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

struct LayerMarkerConfig {
  std::string topic;
  std::string ns;
  float marker_scale = 0.1f;
};

template <typename LayerT, typename ColorPolicyT>
struct NamedDiscreteMarkerLayer {
  std::string name;
  const LayerT& layer;
  ColorPolicyT color_policy;
  LayerMarkerConfig marker_config;
};

template <typename LayerT, typename ColorPolicyT>
NamedDiscreteMarkerLayer<LayerT, ColorPolicyT> namedDiscreteMarkerLayer(
    std::string name, const LayerT& layer, ColorPolicyT color_policy,
    LayerMarkerConfig marker_config = {}) {
  if (marker_config.topic.empty()) {
    marker_config.topic = "/wavemap/discrete/" + name;
  }
  if (marker_config.ns.empty()) {
    marker_config.ns = name;
  }
  return {std::move(name), layer, std::move(color_policy),
          std::move(marker_config)};
}

template <typename DiscreteLayersT>
struct DiscreteLayerMarkerTraits {
  static auto layers(const DiscreteLayersT&) {
    static_assert(detail::kAlwaysFalse<DiscreteLayersT>,
                  "DiscreteLayerMarkerTraits<T> must be specialized for this "
                  "discrete layer bundle type.");
  }
};

template <typename ValueT, typename ColorPolicyT>
bool discreteLayerToMarkerArray(const std::string& frame_id,
                                const ros::Time& stamp,
                                const wavemap::layered::DiscreteLayer<ValueT>& layer,
                                const ColorPolicyT& color_policy,
                                const LayerMarkerConfig& config,
                                visualization_msgs::MarkerArray& msg) {
  msg.markers.clear();

  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = config.ns;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = config.marker_scale;
  marker.scale.y = config.marker_scale;
  marker.scale.z = config.marker_scale;

  for (const auto& [parent_key, cell] : layer.cells()) {
    for (const int offset : cell.observed_offsets) {
      const auto exception_it = cell.exceptions.find(offset);
      const ValueT value = exception_it != cell.exceptions.end()
                               ? exception_it->second
                               : cell.dominant_value;

      std_msgs::ColorRGBA voxel_color;
      if (!color_policy(value, voxel_color)) {
        continue;
      }

      const int side_length = layer.config().blockSideLength();
      const int local_x = offset % side_length;
      const int local_y = (offset / side_length) % side_length;
      const int local_z = offset / (side_length * side_length);

      geometry_msgs::Point point;
      point.x = (parent_key.x * side_length + local_x) * config.marker_scale;
      point.y = (parent_key.y * side_length + local_y) * config.marker_scale;
      point.z = (parent_key.z * side_length + local_z) * config.marker_scale;
      marker.points.emplace_back(point);
      marker.colors.emplace_back(voxel_color);
    }
  }

  if (!marker.points.empty()) {
    msg.markers.emplace_back(std::move(marker));
  }
  return true;
}

template <typename DiscreteLayersT, typename PublisherFactoryT>
bool publishDiscreteLayerMarkers(const DiscreteLayersT& layers,
                                 const std::string& frame_id,
                                 const ros::Time& stamp,
                                 PublisherFactoryT&& publisher_factory) {
  const auto named_layers = DiscreteLayerMarkerTraits<DiscreteLayersT>::layers(layers);
  return detail::forEachTupleElement(named_layers, [&](const auto& named_layer) {
    visualization_msgs::MarkerArray marker_msg;
    if (!discreteLayerToMarkerArray(frame_id, stamp, named_layer.layer,
                                    named_layer.color_policy,
                                    named_layer.marker_config, marker_msg)) {
      return false;
    }
    publisher_factory(named_layer.marker_config.topic).publish(marker_msg);
    return true;
  });
}
}  // namespace layered_map_viz

#endif  // WAVEMAP_ROS_CONVERSIONS_DISCRETE_LAYER_MARKER_CONVERSIONS_H_
