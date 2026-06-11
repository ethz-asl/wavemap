#ifndef WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_LAYERED_ROS_CONVERTER_H_
#define WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_LAYERED_ROS_CONVERTER_H_

#include <string>
#include <vector>

#include <wavemap_msgs/Layer.h>

#include "layered_voxel_config.h"

struct LayeredVoxelRosConverter {
  static std::vector<std::string> layerNames() {
    // These names describe the custom data carried inside each LayeredVoxel.
    return {"color", "traversability"};
  }

  static std::vector<std::string> layerTypes() {
    // The type strings tell consumers how to interpret each layer's arrays.
    return {"float32_rgb", "float32"};
  }

  static std::vector<wavemap_msgs::Layer> makeLayers() {
    // The message stores custom values layer-major: all color values go into the color layer, and all traversability values go into the traversability layer.
    wavemap_msgs::Layer color_layer;
    color_layer.name = "color";
    color_layer.type = "float32_rgb";

    wavemap_msgs::Layer traversability_layer;
    traversability_layer.name = "traversability";
    traversability_layer.type = "float32";

    return {color_layer, traversability_layer};
  }

  static void appendLayerValues(const LayeredVoxel& voxel, std::vector<wavemap_msgs::Layer>& layers) {
    // Color uses three float values per voxel/coefficient: r, g, and b.
    layers[0].float32_values.emplace_back(voxel.data.r);
    layers[0].float32_values.emplace_back(voxel.data.g);
    layers[0].float32_values.emplace_back(voxel.data.b);

    // Traversability uses one float value per voxel/coefficient.
    layers[1].float32_values.emplace_back(voxel.data.traversability);
  }

  static LayeredVoxel readCellData(wavemap::FloatingPoint occupancy, const std::vector<wavemap_msgs::Layer>& layers, size_t value_index) {
    // The value_index selects which voxel/coefficient is being reconstructed inside each layer-major array.
    const size_t color_index = 3u * value_index;
    return LayeredVoxel(occupancy, LayeredData{layers[0].float32_values[color_index], layers[0].float32_values[color_index + 1u], layers[0].float32_values[color_index + 2u], layers[1].float32_values[value_index]});
  }
};

inline void populateLayeredCube(LayeredMap& map, std::vector<wavemap::Index3D>& populated_indices) {
  const wavemap::Index3D cube_origin(8, 2, 2);
  constexpr int cube_size_x = 2;
  constexpr int cube_size_y = 2;
  constexpr int cube_size_z = 4;

  size_t populated_voxels = 0u;
  populated_indices.reserve(cube_size_x * cube_size_y * cube_size_z);

  // Populate a small cube so conversions and publishing touch several wavelet coefficients instead of a single trivial voxel.
  for (int dx = 0; dx < cube_size_x; ++dx) {
    for (int dy = 0; dy < cube_size_y; ++dy) {
      for (int dz = 0; dz < cube_size_z; ++dz) {
        const wavemap::Index3D voxel_index = cube_origin + wavemap::Index3D(dx, dy, dz);
        const wavemap::FloatingPoint layer_value = static_cast<wavemap::FloatingPoint>(populated_voxels);
        const LayeredVoxel original_voxel(0.5f + 0.02f * layer_value, LayeredData{0.1f * dx, 0.1f * dy, 0.05f * dz, 0.9f - 0.03f * layer_value});
        const LayeredVoxel voxel_update(0.1f, LayeredData{0.01f, 0.02f, 0.03f, -0.01f});

        map.setVoxelValue(voxel_index, original_voxel);
        map.addToVoxelValue(voxel_index, voxel_update);
        populated_indices.emplace_back(voxel_index);
        ++populated_voxels;
      }
    }
  }
}

#endif  // WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_LAYERED_ROS_CONVERTER_H_
