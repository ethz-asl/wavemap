#ifndef WAVEMAP_ROS_CONVERSIONS_DESCRIPTOR_LAYER_OBSERVATION_CONVERSIONS_H_
#define WAVEMAP_ROS_CONVERSIONS_DESCRIPTOR_LAYER_OBSERVATION_CONVERSIONS_H_

#include <algorithm>
#include <string>
#include <tuple>
#include <type_traits>
#include <vector>

#include <wavemap/layered/integration/layered_pipeline.h>
#include <wavemap_msgs/LayeredMapUpdate.h>
#include <wavemap_ros_conversions/descriptor_layered_map_conversions.h>
#include <wavemap_ros_conversions/geometry_msg_conversions.h>

namespace wavemap::convert {
namespace detail {

template <typename ValueT>
bool layerPayloadIsValid(const wavemap_msgs::Layer& layer,
                         size_t observation_count) {
  return layer.type == LayerRosCodec<ValueT>::typeName() &&
         LayerRosCodec<ValueT>::payloadIsValid(layer, observation_count);
}

template <typename LayerTagT, typename PipelineT>
bool integrateLayer(
    const wavemap_msgs::Layer& layer,
    const std::vector<Point3D>& positions, PipelineT& pipeline,
    bool apply, std::string* error_message) {
  using Value = layered::LayerValueT<LayerTagT>;
  if (!layerPayloadIsValid<Value>(layer, positions.size())) {
    if (error_message) {
      *error_message = "Layer '" + layer.name +
                       "' has the wrong type or value count.";
    }
    return false;
  }

  if (!apply) {
    return true;
  }

  std::vector<layered::LayerObservation<Value>> observations;
  observations.reserve(positions.size());
  for (size_t index = 0u; index < positions.size(); ++index) {
    observations.emplace_back(
        positions[index], LayerRosCodec<Value>::read(layer, index));
  }
  const auto result = pipeline.template integrate<LayerTagT>(observations);
  if (result.rejected != 0u) {
    if (error_message) {
      *error_message = "Layer '" + layer.name +
                       "' contains an invalid spatial observation.";
    }
    return false;
  }
  return true;
}

template <typename BundleT, typename PipelineT>
bool integrateBundle(
    const wavemap_msgs::LayeredMapUpdate& msg,
    const std::vector<Point3D>& positions, PipelineT& pipeline,
    std::vector<bool>& consumed, bool apply, std::string* error_message) {
  bool success = true;
  const auto descriptors = layered::layerDescriptors<BundleT>();
  if constexpr (std::tuple_size_v<decltype(descriptors)> == 0u) {
    (void)msg;
    (void)positions;
    (void)pipeline;
    (void)consumed;
    (void)apply;
    (void)error_message;
  } else {
    std::apply(
        [&](const auto&... descriptor) {
          auto process_descriptor = [&](const auto& current_descriptor) {
            using Descriptor = std::decay_t<decltype(current_descriptor)>;
            using LayerTag = typename Descriptor::LayerTag;
            size_t match_count = 0u;
            size_t match_index = 0u;
            for (size_t index = 0u; index < msg.layers.size(); ++index) {
              if (msg.layers[index].name == current_descriptor.name()) {
                ++match_count;
                match_index = index;
              }
            }
            if (match_count > 1u) {
              success = false;
              if (error_message) {
                *error_message = "Layer '" +
                                 std::string(current_descriptor.name()) +
                                 "' occurs more than once.";
              }
              return;
            }
            if (match_count == 1u) {
              consumed[match_index] = true;
              success = success &&
                        integrateLayer<LayerTag>(
                            msg.layers[match_index], positions, pipeline,
                            apply, error_message);
            }
          };
          (process_descriptor(descriptor), ...);
        },
        descriptors);
  }
  return success;
}

}  // namespace detail

template <typename LayeredMapT>
struct DescriptorLayerObservationRosConverter {
  using Pipeline = layered::LayeredPipeline<LayeredMapT>;

  static bool integrate(const wavemap_msgs::LayeredMapUpdate& msg,
                        Pipeline& pipeline,
                        std::string* error_message = nullptr) {
    if (msg.positions.empty() || msg.layers.empty()) {
      if (error_message) {
        *error_message =
            "A layered map update requires positions and at least one layer.";
      }
      return false;
    }

    std::vector<Point3D> positions;
    positions.reserve(msg.positions.size());
    for (const auto& position_msg : msg.positions) {
      const Point3D position = point32MsgToPoint3D(position_msg);
      if (!position.array().isFinite().all()) {
        if (error_message) {
          *error_message = "A layered map update position is not finite.";
        }
        return false;
      }
      positions.emplace_back(position);
    }

    std::vector<bool> consumed(msg.layers.size(), false);
    if (!detail::integrateBundle<typename LayeredMapT::ContinuousLayers>(
            msg, positions, pipeline, consumed, false, error_message) ||
        !detail::integrateBundle<typename LayeredMapT::DiscreteLayers>(
            msg, positions, pipeline, consumed, false, error_message)) {
      return false;
    }

    const auto unknown = std::find(consumed.cbegin(), consumed.cend(), false);
    if (unknown != consumed.cend()) {
      const size_t index = std::distance(consumed.cbegin(), unknown);
      if (error_message) {
        *error_message = "Unknown layer '" + msg.layers[index].name + "'.";
      }
      return false;
    }

    std::fill(consumed.begin(), consumed.end(), false);
    return detail::integrateBundle<typename LayeredMapT::ContinuousLayers>(
               msg, positions, pipeline, consumed, true, error_message) &&
           detail::integrateBundle<typename LayeredMapT::DiscreteLayers>(
               msg, positions, pipeline, consumed, true, error_message);
  }
};

}  // namespace wavemap::convert

#endif  // WAVEMAP_ROS_CONVERSIONS_DESCRIPTOR_LAYER_OBSERVATION_CONVERSIONS_H_
