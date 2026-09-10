#include <algorithm>
#include <string>

#include <OGRE/OgreColourValue.h>
#include <pluginlib/class_list_macros.h>
#include <wavemap_ros_conversions/descriptor_layered_map_conversions.h>

#include "occupancy_map_layers.h"
#include "reflectivity_map_layers.h"
#include "reflectivity_only_map_layers.h"
#include "rgb_map_layers.h"
#include "signal_map_layers.h"
#include "wavemap_rviz_plugin/layered_map_factory.h"

namespace wavemap::rviz_plugin {
namespace occupancy_map = wavemap::examples::occupancy_map;
namespace reflectivity_map = wavemap::examples::reflectivity_map;
namespace reflectivity_only_map = wavemap::examples::reflectivity_only_map;
namespace rgb_map = wavemap::examples::rgb_map;
namespace signal_map = wavemap::examples::signal_map;

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

Ogre::ColourValue scalarGradient(FloatingPoint normalized_value,
                                 const Ogre::ColourValue& low,
                                 const Ogre::ColourValue& high) {
  const FloatingPoint value = std::clamp(normalized_value, 0.f, 1.f);
  return Ogre::ColourValue(low.r + value * (high.r - low.r),
                           low.g + value * (high.g - low.g),
                           low.b + value * (high.b - low.b), 1.f);
}

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

struct SignalContinuousLayerColorProvider {
  template <typename LayerTagT, typename VoxelT>
  static Ogre::ColourValue colorFor(
      const VoxelT& voxel, FloatingPoint scalar_display_min,
      FloatingPoint scalar_display_max,
      const Ogre::ColourValue& scalar_low_color,
      const Ogre::ColourValue& scalar_high_color) {
    LayerMetadata display_metadata;
    display_metadata.min_values = {scalar_display_min};
    display_metadata.max_values = {scalar_display_max};
    const auto& state = voxel.data.template get<LayerTagT>();
    const FloatingPoint value = normalizeScalar(
        layered::LayerStateConversion<LayerTagT>::value(state),
        display_metadata);
    return scalarGradient(value, scalar_low_color, scalar_high_color);
  }

  template <typename VoxelT>
  static bool getLayerColor(
      const std::string& layer_name, const VoxelT& voxel,
      FloatingPoint /*occupancy*/, FloatingPoint scalar_display_min,
      FloatingPoint scalar_display_max,
      const Ogre::ColourValue& scalar_low_color,
      const Ogre::ColourValue& scalar_high_color,
      const LayerMetadata& /*metadata*/, Ogre::ColourValue& color) {
    if (layer_name == "reflectivity") {
      color = colorFor<signal_map::ReflectivityLayer>(
          voxel, scalar_display_min, scalar_display_max, scalar_low_color,
          scalar_high_color);
    } else if (layer_name == "signal") {
      color = colorFor<signal_map::SignalLayer>(
          voxel, scalar_display_min, scalar_display_max, scalar_low_color,
          scalar_high_color);
    } else if (layer_name == "near_ir") {
      color = colorFor<signal_map::NearIrLayer>(
          voxel, scalar_display_min, scalar_display_max, scalar_low_color,
          scalar_high_color);
    } else {
      return false;
    }
    return true;
  }
};

struct RgbContinuousLayerColorProvider {
  template <typename VoxelT>
  static bool getLayerColor(
      const std::string& layer_name, const VoxelT& voxel,
      FloatingPoint /*occupancy*/, FloatingPoint /*scalar_display_min*/,
      FloatingPoint /*scalar_display_max*/,
      const Ogre::ColourValue& /*scalar_low_color*/,
      const Ogre::ColourValue& /*scalar_high_color*/,
      const LayerMetadata& /*metadata*/, Ogre::ColourValue& color) {
    if (layer_name != "rgb") {
      return false;
    }
    const layered::Rgb value = layered::LayerStateConversion<
        rgb_map::RgbLayer>::value(
        voxel.data.template get<rgb_map::RgbLayer>());
    color = Ogre::ColourValue(std::clamp(value.r, 0.f, 1.f),
                              std::clamp(value.g, 0.f, 1.f),
                              std::clamp(value.b, 0.f, 1.f), 1.f);
    return true;
  }
};

signal_map::NearIrClassDefinition::Config makeAllLayersH2MapConfig() {
  return signal_map::makeMapConfigFor<
      signal_map::NearIrClassDefinition>(2);
}

reflectivity_map::Definition::Config makeReflectivityClassMapConfig() {
  return reflectivity_map::makeMapConfig<
      reflectivity_map::Definition>(0);
}

class OccupancyLayeredMapFactory final
    : public TypedLayeredMapFileFactory<
          occupancy_map::Definition::ContinuousMap,
          wavemap::convert::DescriptorContinuousRosConverter<
              occupancy_map::Definition::Voxel>,
          NoLayerColorProvider, occupancy_map::Definition::Map,
          occupancy_map::Definition::MapIo,
          occupancy_map::makeMapConfig<occupancy_map::Definition>> {};

class ReflectivityLayeredMapFactory final
    : public TypedLayeredMapFileFactory<
          reflectivity_only_map::Definition::ContinuousMap,
          wavemap::convert::DescriptorContinuousRosConverter<
              reflectivity_only_map::Definition::Voxel>,
          ReflectivityContinuousLayerColorProvider,
          reflectivity_only_map::Definition::Map,
          reflectivity_only_map::Definition::MapIo,
          reflectivity_only_map::makeMapConfig<
              reflectivity_only_map::Definition>> {};

class ReflectivityClassLayeredMapFactory final
    : public TypedLayeredMapFileFactory<
          reflectivity_map::Definition::ContinuousMap,
          wavemap::convert::DescriptorContinuousRosConverter<
              reflectivity_map::Definition::Voxel>,
          ReflectivityContinuousLayerColorProvider,
          reflectivity_map::Definition::Map,
          reflectivity_map::Definition::MapIo,
          makeReflectivityClassMapConfig> {};

class SignalNearIrClassLayeredMapFactory final
    : public TypedLayeredMapFileFactory<
          signal_map::NearIrClassDefinition::ContinuousMap,
          wavemap::convert::DescriptorContinuousRosConverter<
              signal_map::NearIrClassDefinition::Voxel>,
          SignalContinuousLayerColorProvider,
          signal_map::NearIrClassDefinition::Map,
          signal_map::NearIrClassDefinition::MapIo,
          makeAllLayersH2MapConfig> {};

class RgbLayeredMapFactory final
    : public TypedLayeredMapFileFactory<
          rgb_map::Definition::ContinuousMap,
          wavemap::convert::DescriptorContinuousRosConverter<
              rgb_map::Definition::Voxel>,
          RgbContinuousLayerColorProvider, rgb_map::Definition::Map,
          rgb_map::Definition::MapIo, rgb_map::makeMapConfig> {};
}  // namespace wavemap::rviz_plugin

PLUGINLIB_EXPORT_CLASS(wavemap::rviz_plugin::OccupancyLayeredMapFactory,
                       wavemap::rviz_plugin::LayeredMapFactory)
PLUGINLIB_EXPORT_CLASS(wavemap::rviz_plugin::ReflectivityLayeredMapFactory,
                       wavemap::rviz_plugin::LayeredMapFactory)
PLUGINLIB_EXPORT_CLASS(wavemap::rviz_plugin::ReflectivityClassLayeredMapFactory,
                       wavemap::rviz_plugin::LayeredMapFactory)
PLUGINLIB_EXPORT_CLASS(
    wavemap::rviz_plugin::SignalNearIrClassLayeredMapFactory,
    wavemap::rviz_plugin::LayeredMapFactory)
PLUGINLIB_EXPORT_CLASS(wavemap::rviz_plugin::RgbLayeredMapFactory,
                       wavemap::rviz_plugin::LayeredMapFactory)
