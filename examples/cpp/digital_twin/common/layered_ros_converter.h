#ifndef WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_LAYERED_ROS_CONVERTER_H_
#define WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_LAYERED_ROS_CONVERTER_H_

#include <string>
#include <vector>

#include <wavemap_msgs/Layer.h>
#include <wavemap_msgs/LayerVisualization.h>

#include "layered_voxel_config.h"

struct LayeredVoxelRosConverter {
  static std::vector<std::string> layerNames() {
    std::vector<std::string> names;
    for (const auto& layer :
         wavemap::layered::ContinuousLayerSchemaTraits<ContinuousLayers>::layers()) {
      names.emplace_back(layer.name);
    }
    return names;
  }

  static std::vector<std::string> layerTypes() {
    std::vector<std::string> types;
    for (const auto& layer :
         wavemap::layered::ContinuousLayerSchemaTraits<ContinuousLayers>::layers()) {
      types.emplace_back(layer.type);
    }
    return types;
  }

  static std::vector<wavemap_msgs::LayerVisualization> layerVisualizations() {
    return {};
  }

  static std::vector<wavemap_msgs::Layer> makeLayers() {
    // The message stores custom values layer-major: all color values go into
    // the color layer, and all traversability values go into the traversability
    // layer.
    std::vector<wavemap_msgs::Layer> layers;
    for (const auto& layer_schema :
         wavemap::layered::ContinuousLayerSchemaTraits<ContinuousLayers>::layers()) {
      auto& layer = layers.emplace_back();
      layer.name = layer_schema.name;
      layer.type = layer_schema.type;
    }
    return layers;
  }

  static std::vector<wavemap_msgs::Layer> makeLayerMinimums(
      const ContinuousWaveletMap::ThresholdConfig& config) {
    auto layers = makeLayers();
    layers[0].float32_values = {config.data.rgb_min.r, config.data.rgb_min.g,
                                config.data.rgb_min.b};
    layers[1].float32_values = {config.data.traversability_min};
    return layers;
  }

  static std::vector<wavemap_msgs::Layer> makeLayerMaximums(
      const ContinuousWaveletMap::ThresholdConfig& config) {
    auto layers = makeLayers();
    layers[0].float32_values = {config.data.rgb_max.r, config.data.rgb_max.g,
                                config.data.rgb_max.b};
    layers[1].float32_values = {config.data.traversability_max};
    return layers;
  }

  static void appendLayerValues(const LayeredVoxel& voxel,
                                std::vector<wavemap_msgs::Layer>& layers) {
    // Color uses three float values per voxel/coefficient: r, g, and b.
    layers[0].float32_values.emplace_back(voxel.data.rgb.r);
    layers[0].float32_values.emplace_back(voxel.data.rgb.g);
    layers[0].float32_values.emplace_back(voxel.data.rgb.b);

    // Traversability uses one float value per voxel/coefficient.
    layers[1].float32_values.emplace_back(voxel.data.traversability);
  }

  static LayeredVoxel readCellData(
      wavemap::FloatingPoint occupancy,
      const std::vector<wavemap_msgs::Layer>& layers, size_t value_index) {
    // The value_index selects which voxel/coefficient is being reconstructed
    // inside each layer-major array.
    const size_t color_index = 3u * value_index;
    const Rgb color = rgb(layers[0].float32_values[color_index],
                          layers[0].float32_values[color_index + 1u],
                          layers[0].float32_values[color_index + 2u]);
    return LayeredVoxel(
        occupancy,
        ContinuousLayers{color, layers[1].float32_values[value_index]});
  }
};

inline void populateLayeredCube(
    ContinuousWaveletMap& map, std::vector<wavemap::Index3D>& populated_indices) {
  const wavemap::Index3D cube_origin(8, 2, 2);
  constexpr int cube_size_x = 2;
  constexpr int cube_size_y = 2;
  constexpr int cube_size_z = 4;

  size_t populated_voxels = 0u;
  populated_indices.reserve(cube_size_x * cube_size_y * cube_size_z);

  // Populate a small cube so conversions and publishing touch several wavelet
  // coefficients instead of a single trivial voxel.
  for (int dx = 0; dx < cube_size_x; ++dx) {
    for (int dy = 0; dy < cube_size_y; ++dy) {
      for (int dz = 0; dz < cube_size_z; ++dz) {
        const wavemap::Index3D voxel_index =
            cube_origin + wavemap::Index3D(dx, dy, dz);
        const wavemap::FloatingPoint layer_value =
            static_cast<wavemap::FloatingPoint>(populated_voxels);
        const Rgb visible_color = rgb(
            dx == 0 ? 0.15f : 1.0f,
            dy == 0 ? 0.15f : 1.0f,
            0.25f + 0.25f * static_cast<float>(dz));
        const LayeredVoxel original_voxel(
            0.5f + 0.02f * layer_value,
            ContinuousLayers{visible_color, 0.9f - 0.03f * layer_value});
        const LayeredVoxel voxel_update(
            0.1f,
            ContinuousLayers{rgb(0.f, 0.f, 0.f), -0.01f});

        map.setVoxelValue(voxel_index, original_voxel);
        map.addToVoxelValue(voxel_index, voxel_update);
        populated_indices.emplace_back(voxel_index);
        ++populated_voxels;
      }
    }
  }
}

#endif  // WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_LAYERED_ROS_CONVERTER_H_
