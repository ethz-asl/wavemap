#ifndef WAVEMAP_ROS_CONVERSIONS_DESCRIPTOR_LAYERED_MAP_CONVERSIONS_H_
#define WAVEMAP_ROS_CONVERSIONS_DESCRIPTOR_LAYERED_MAP_CONVERSIONS_H_

#include <string>
#include <type_traits>
#include <vector>

#include <wavemap/layered/schema/layer_descriptor.h>
#include <wavemap/layered/schema/schema_layer_storage.h>
#include <wavemap/layered/types/rgb.h>
#include <wavemap_msgs/Layer.h>
#include <wavemap_ros_conversions/layered_map_msg_conversions.h>
#include <wavemap_msgs/LayerVisualization.h>

namespace wavemap::convert {

template <typename ValueT>
struct LayerRosCodec;

namespace detail {
template <typename ValueT, typename = void>
struct HasLayerRosCodec : std::false_type {};

template <typename ValueT>
struct HasLayerRosCodec<
    ValueT,
    std::void_t<decltype(LayerRosCodec<ValueT>::typeName()),
                decltype(LayerRosCodec<ValueT>::append(
                    std::declval<const ValueT&>(),
                    std::declval<wavemap_msgs::Layer&>())),
                decltype(LayerRosCodec<ValueT>::read(
                    std::declval<const wavemap_msgs::Layer&>(), size_t{})),
                decltype(LayerRosCodec<ValueT>::payloadIsValid(
                    std::declval<const wavemap_msgs::Layer&>(), size_t{}))>>
    : std::true_type {};
}  // namespace detail

template <typename ValueT>
inline constexpr bool kHasLayerRosCodec =
    detail::HasLayerRosCodec<ValueT>::value;

template <>
struct LayerRosCodec<FloatingPoint> {
  static constexpr const char* typeName() { return "float32"; }

  static void append(const FloatingPoint& value, wavemap_msgs::Layer& layer) {
    layer.float32_values.emplace_back(value);
  }

  static FloatingPoint read(const wavemap_msgs::Layer& layer,
                            size_t value_index) {
    return layer.float32_values[value_index];
  }

  static bool payloadIsValid(const wavemap_msgs::Layer& layer,
                             size_t value_count) {
    return layer.float32_values.size() == value_count &&
           layer.int32_values.empty() && layer.uint8_values.empty();
  }
};

template <>
struct LayerRosCodec<layered::Rgb> {
  static constexpr const char* typeName() { return "float32_rgb"; }

  static void append(const layered::Rgb& value,
                     wavemap_msgs::Layer& layer) {
    layer.float32_values.emplace_back(value.r);
    layer.float32_values.emplace_back(value.g);
    layer.float32_values.emplace_back(value.b);
  }

  static layered::Rgb read(const wavemap_msgs::Layer& layer,
                           size_t value_index) {
    const size_t first_component = 3u * value_index;
    return {layer.float32_values[first_component],
            layer.float32_values[first_component + 1u],
            layer.float32_values[first_component + 2u]};
  }

  static bool payloadIsValid(const wavemap_msgs::Layer& layer,
                             size_t value_count) {
    return layer.float32_values.size() == 3u * value_count &&
           layer.int32_values.empty() && layer.uint8_values.empty();
  }
};

template <>
struct LayerRosCodec<int> {
  static constexpr const char* typeName() { return "int32"; }

  static void append(int value, wavemap_msgs::Layer& layer) {
    layer.int32_values.emplace_back(value);
  }

  static int read(const wavemap_msgs::Layer& layer, size_t value_index) {
    return layer.int32_values[value_index];
  }

  static bool payloadIsValid(const wavemap_msgs::Layer& layer,
                             size_t value_count) {
    return layer.int32_values.size() == value_count &&
           layer.float32_values.empty() && layer.uint8_values.empty();
  }
};

template <typename VoxelT>
struct DescriptorContinuousRosConverter {
  using ContinuousLayers = typename VoxelT::AdditionalData;

  static_assert(
      []() constexpr {
        bool supported = true;
        const auto descriptors = layered::layerDescriptors<ContinuousLayers>();
        std::apply(
            [&](const auto&... descriptor) {
              ((supported = supported &&
                            kHasLayerRosCodec<layered::LayerValueT<
                                typename std::decay_t<
                                    decltype(descriptor)>::LayerTag>>),
               ...);
            },
            descriptors);
        return supported;
      }(),
      "Every continuous layer value published through ROS needs a "
      "LayerRosCodec specialization.");

  static std::vector<std::string> layerNames() {
    std::vector<std::string> names;
    const auto descriptors = layered::layerDescriptors<ContinuousLayers>();
    names.reserve(std::tuple_size_v<decltype(descriptors)>);
    std::apply(
        [&](const auto&... descriptor) {
          (names.emplace_back(descriptor.name()), ...);
        },
        descriptors);
    return names;
  }

  static std::vector<std::string> layerTypes() {
    std::vector<std::string> types;
    const auto descriptors = layered::layerDescriptors<ContinuousLayers>();
    types.reserve(std::tuple_size_v<decltype(descriptors)>);
    std::apply(
        [&](const auto&... descriptor) {
          (types.emplace_back(
               LayerRosCodec<layered::LayerValueT<
                   typename std::decay_t<decltype(descriptor)>::LayerTag>>::
                   typeName()),
           ...);
        },
        descriptors);
    return types;
  }

  static std::vector<wavemap_msgs::LayerVisualization> layerVisualizations() {
    std::vector<wavemap_msgs::LayerVisualization> visualizations;
    const auto descriptors = layered::layerDescriptors<ContinuousLayers>();
    visualizations.reserve(std::tuple_size_v<decltype(descriptors)>);
    std::apply(
        [&](const auto&... descriptor) {
          ([&]() {
            using LayerTag =
                typename std::decay_t<decltype(descriptor)>::LayerTag;
            auto& msg = visualizations.emplace_back();
            msg.name = descriptor.name();
            if (const auto visualization =
                    layered::scalarLayerVisualization<LayerTag>()) {
              msg.has_low_color = true;
              msg.low_r = visualization->low.r;
              msg.low_g = visualization->low.g;
              msg.low_b = visualization->low.b;
              msg.has_high_color = true;
              msg.high_r = visualization->high.r;
              msg.high_g = visualization->high.g;
              msg.high_b = visualization->high.b;
            }
          }(), ...);
        }, descriptors);
    return visualizations;

  }
  static std::vector<wavemap_msgs::Layer> makeLayers() {
    std::vector<wavemap_msgs::Layer> layers;
    const auto names = layerNames();
    const auto types = layerTypes();
    layers.resize(names.size());
    for (size_t layer_index = 0u; layer_index < layers.size(); ++layer_index) {
      layers[layer_index].name = names[layer_index];
      layers[layer_index].type = types[layer_index];
    }
    return layers;
  }

  static std::vector<wavemap_msgs::Layer> makeLayerMinimums(
      const typename CellDataTraits<VoxelT>::ThresholdConfig& config) {
    return makeLayerBounds(config, true);
  }

  static std::vector<wavemap_msgs::Layer> makeLayerMaximums(
      const typename CellDataTraits<VoxelT>::ThresholdConfig& config) {
    return makeLayerBounds(config, false);
  }

  static void appendLayerValues(const VoxelT& voxel,
                                std::vector<wavemap_msgs::Layer>& layers) {
    size_t layer_index = 0u;
    layered::forEachLayerDescriptor(
        voxel.data, [&](const auto& descriptor, const auto& state) {
          using LayerTag = typename std::decay_t<decltype(descriptor)>::LayerTag;
          using Value = layered::LayerValueT<LayerTag>;
          LayerRosCodec<Value>::append(
              layered::LayerStateConversion<LayerTag>::value(state),
              layers[layer_index++]);
        });
  }

  static VoxelT readCellData(
      FloatingPoint occupancy, const std::vector<wavemap_msgs::Layer>& layers,
      size_t value_index) {
    VoxelT voxel;
    voxel.occupancy = occupancy;
    size_t layer_index = 0u;
    layered::forEachLayerDescriptor(
        voxel.data, [&](const auto& descriptor, auto& state) {
          using LayerTag = typename std::decay_t<decltype(descriptor)>::LayerTag;
          using Value = layered::LayerValueT<LayerTag>;
          state = layered::LayerStateConversion<LayerTag>::initialState(
              LayerRosCodec<Value>::read(layers[layer_index++], value_index));
        });
    return voxel;
  }

 private:
  static std::vector<wavemap_msgs::Layer> makeLayerBounds(
      const typename CellDataTraits<VoxelT>::ThresholdConfig& config,
      bool minimum) {
    auto layers = makeLayers();
    size_t layer_index = 0u;
    const auto descriptors = layered::layerDescriptors<ContinuousLayers>();
    std::apply(
        [&](const auto&... descriptor) {
          (appendLayerBound(descriptor, config, minimum,
                            layers[layer_index++]),
           ...);
        },
        descriptors);
    return layers;
  }

  template <typename DescriptorT>
  static void appendLayerBound(
      const DescriptorT& /*descriptor*/,
      const typename CellDataTraits<VoxelT>::ThresholdConfig& config,
      bool minimum, wavemap_msgs::Layer& layer) {
    using LayerTag = typename DescriptorT::LayerTag;
    const auto& bounds = config.data.template get<LayerTag>();
    using Value = layered::LayerValueT<LayerTag>;
    LayerRosCodec<Value>::append(minimum ? bounds.min : bounds.max, layer);
  }
};

template <typename DiscreteLayersT>
auto descriptorNamedDiscreteLayersForRos(DiscreteLayersT& layers) {
  return layered::transformLayerDescriptors(
      layered::layerDescriptors<DiscreteLayersT>(),
      [&](const auto& descriptor) {
        using LayerTag = typename std::decay_t<decltype(descriptor)>::LayerTag;
        return namedDiscreteLayerForRos(std::string(descriptor.name()),
                                        descriptor.value(layers),
                                        layered::integerCategoryMetadata<LayerTag>());
      });
}

template <typename DiscreteLayersT>
auto descriptorNamedDiscreteLayersForRos(const DiscreteLayersT& layers) {
  return layered::transformLayerDescriptors(
      layered::layerDescriptors<DiscreteLayersT>(),
      [&](const auto& descriptor) {
        using LayerTag = typename std::decay_t<decltype(descriptor)>::LayerTag;
        return namedDiscreteLayerForRos(std::string(descriptor.name()),
                                        descriptor.value(layers),
                                        layered::integerCategoryMetadata<LayerTag>());
      });
}

// Schema-generated discrete bundles already carry their authoritative names
// and value types in their descriptors. They therefore need no user-written
// ROS serialization registration.
template <typename... LayerTags>
struct DiscreteLayerBundleRosTraits<layered::schema::DiscreteLayerBundle<
    layered::schema::LayerSchema<LayerTags...>>> {
  using Bundle = layered::schema::DiscreteLayerBundle<
      layered::schema::LayerSchema<LayerTags...>>;

  static auto layers(Bundle& layers) {
    return descriptorNamedDiscreteLayersForRos(layers);
  }

  static auto layers(const Bundle& layers) {
    return descriptorNamedDiscreteLayersForRos(layers);
  }
};

}  // namespace wavemap::convert

#endif  // WAVEMAP_ROS_CONVERSIONS_DESCRIPTOR_LAYERED_MAP_CONVERSIONS_H_
