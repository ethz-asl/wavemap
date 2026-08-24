#include <algorithm>
#include <string>

#include <OGRE/OgreColourValue.h>
#include <pluginlib/class_list_macros.h>

#include "example_layered_map_ros_config.h"
#include "field_layered_map_ros_config.h"
#include "layered_ros_converter.h"
#include "occupancy_map_layers.h"
#include "reflectivity_map_layers.h"
#include "reflectivity_only_map_layers.h"
#include "wavemap_rviz_plugin/layered_map_factory.h"

namespace wavemap::rviz_plugin {
namespace field_map = wavemap::examples::field_map;
namespace occupancy_map = wavemap::examples::occupancy_map;
namespace reflectivity_map = wavemap::examples::reflectivity_map;
namespace reflectivity_only_map = wavemap::examples::reflectivity_only_map;

FloatingPoint normalizeScalar(FloatingPoint value,
                              const LayerMetadata& metadata) {
  if (metadata.min_values.size() != 1u ||
      metadata.max_values.size() != 1u ||
      metadata.max_values.front() <= metadata.min_values.front()) {
    return std::clamp(value, 0.f, 1.f);
  }
  return std::clamp((value - metadata.min_values.front()) /
                        (metadata.max_values.front() -
                         metadata.min_values.front()),
                    0.f, 1.f);
}

FloatingPoint normalizeComponent(FloatingPoint value,
                                 const LayerMetadata& metadata,
                                 size_t component) {
  if (component >= metadata.min_values.size() ||
      component >= metadata.max_values.size() ||
      metadata.max_values[component] <= metadata.min_values[component]) {
    return std::clamp(value, 0.f, 1.f);
  }
  return std::clamp((value - metadata.min_values[component]) /
                        (metadata.max_values[component] -
                         metadata.min_values[component]),
                    0.f, 1.f);
}

Ogre::ColourValue scalarGray(FloatingPoint normalized_value) {
  const FloatingPoint visible_value =
      0.15f + 0.85f * std::clamp(normalized_value, 0.f, 1.f);
  return Ogre::ColourValue(visible_value, visible_value, visible_value, 1.f);
}

Ogre::ColourValue scalarGradient(FloatingPoint normalized_value,
                                 const Ogre::ColourValue& low,
                                 const Ogre::ColourValue& high) {
  const FloatingPoint value = std::clamp(normalized_value, 0.f, 1.f);
  return Ogre::ColourValue(low.r + value * (high.r - low.r),
                           low.g + value * (high.g - low.g),
                           low.b + value * (high.b - low.b), 1.f);
}

struct ExampleContinuousLayerColorProvider {
  static bool getLayerColor(const std::string& layer_name,
                            const LayeredVoxel& voxel,
                            FloatingPoint /*occupancy*/,
                            FloatingPoint /*scalar_display_min*/,
                            FloatingPoint /*scalar_display_max*/,
                            const Ogre::ColourValue& /*scalar_low_color*/,
                            const Ogre::ColourValue& /*scalar_high_color*/,
                            const LayerMetadata& metadata,
                            Ogre::ColourValue& color) {
    if (layer_name == "color") {
      color = Ogre::ColourValue(
          normalizeComponent(voxel.data.rgb.r, metadata, 0u),
          normalizeComponent(voxel.data.rgb.g, metadata, 1u),
          normalizeComponent(voxel.data.rgb.b, metadata, 2u), 1.f);
      return true;
    }
    if (layer_name == "traversability") {
      const FloatingPoint value =
          normalizeScalar(voxel.data.traversability, metadata);
      color = scalarGray(value);
      return true;
    }
    return false;
  }
};

struct FieldContinuousLayerColorProvider {
  template <typename VoxelT>
  static bool getLayerColor(const std::string& layer_name,
                            const VoxelT& voxel,
                            FloatingPoint /*occupancy*/,
                            FloatingPoint scalar_display_min,
                            FloatingPoint scalar_display_max,
                            const Ogre::ColourValue& scalar_low_color,
                            const Ogre::ColourValue& scalar_high_color,
                            const LayerMetadata& /*metadata*/,
                            Ogre::ColourValue& color) {
    if (layer_name != "reflectivity") {
      return false;
    }
    LayerMetadata display_metadata;
    display_metadata.min_values = {scalar_display_min};
    display_metadata.max_values = {scalar_display_max};
    const FloatingPoint value = normalizeScalar(
        voxel.data.template get<field_map::ReflectivityLayer>(),
        display_metadata);
    color = scalarGradient(value, scalar_low_color, scalar_high_color);
    return true;
  }
};

struct ReflectivityContinuousLayerColorProvider {
  template <typename VoxelT>
  static bool getLayerColor(const std::string& layer_name,
                            const VoxelT& voxel,
                            FloatingPoint /*occupancy*/,
                            FloatingPoint scalar_display_min,
                            FloatingPoint scalar_display_max,
                            const Ogre::ColourValue& scalar_low_color,
                            const Ogre::ColourValue& scalar_high_color,
                            const LayerMetadata& /*metadata*/,
                            Ogre::ColourValue& color) {
    if (layer_name != "reflectivity") {
      return false;
    }
    LayerMetadata display_metadata;
    display_metadata.min_values = {scalar_display_min};
    display_metadata.max_values = {scalar_display_max};
    const auto& state =
        voxel.data.template get<reflectivity_map::ReflectivityLayer>();
    const FloatingPoint value = normalizeScalar(
        layered::LayerStateConversion<
            reflectivity_map::ReflectivityLayer>::value(state),
        display_metadata);
    color = scalarGradient(value, scalar_low_color, scalar_high_color);
    return true;
  }
};

class ExampleLayeredMapFactory final
    : public TypedLayeredMapFileFactory<
          ContinuousWaveletMap, LayeredVoxelRosConverter,
          ExampleContinuousLayerColorProvider, ExampleLayeredMap,
          ExampleLayeredMapIo,
          layered_map_config::makeDefaultExampleLayeredMapConfig> {};

class FieldLayeredMapFactory final
    : public TypedLayeredMapFileFactory<
          field_map::ContinuousMap,
          field_map::ros_config::VoxelRosConverter,
          FieldContinuousLayerColorProvider, field_map::Map, field_map::MapIo,
          field_map::makeDefaultConfig> {};

class OccupancyLayeredMapFactory final
    : public TypedLayeredMapFileFactory<
          occupancy_map::Definition::ContinuousMap,
          wavemap::convert::DescriptorContinuousRosConverter<
              occupancy_map::Definition::Voxel>,
          NoLayerColorProvider, occupancy_map::Definition::Map,
          occupancy_map::Definition::MapIo, occupancy_map::makeMapConfig> {};

class ReflectivityLayeredMapFactory final
    : public TypedLayeredMapFileFactory<
          reflectivity_only_map::Definition::ContinuousMap,
          wavemap::convert::DescriptorContinuousRosConverter<
              reflectivity_only_map::Definition::Voxel>,
          ReflectivityContinuousLayerColorProvider,
          reflectivity_only_map::Definition::Map,
          reflectivity_only_map::Definition::MapIo,
          reflectivity_only_map::makeMapConfig> {};

class ReflectivityClassLayeredMapFactory final
    : public TypedLayeredMapFileFactory<
          reflectivity_map::Definition::ContinuousMap,
          wavemap::convert::DescriptorContinuousRosConverter<
              reflectivity_map::Definition::Voxel>,
          ReflectivityContinuousLayerColorProvider,
          reflectivity_map::Definition::Map,
          reflectivity_map::Definition::MapIo,
          reflectivity_map::makeMapConfig> {};
}  // namespace wavemap::rviz_plugin

PLUGINLIB_EXPORT_CLASS(wavemap::rviz_plugin::ExampleLayeredMapFactory,
                       wavemap::rviz_plugin::LayeredMapFactory)
PLUGINLIB_EXPORT_CLASS(wavemap::rviz_plugin::FieldLayeredMapFactory,
                       wavemap::rviz_plugin::LayeredMapFactory)
PLUGINLIB_EXPORT_CLASS(wavemap::rviz_plugin::OccupancyLayeredMapFactory,
                       wavemap::rviz_plugin::LayeredMapFactory)
PLUGINLIB_EXPORT_CLASS(wavemap::rviz_plugin::ReflectivityLayeredMapFactory,
                       wavemap::rviz_plugin::LayeredMapFactory)
PLUGINLIB_EXPORT_CLASS(wavemap::rviz_plugin::ReflectivityClassLayeredMapFactory,
                       wavemap::rviz_plugin::LayeredMapFactory)
