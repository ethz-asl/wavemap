#ifndef WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_LAYERED_VOXEL_CONFIG_H_
#define WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_LAYERED_VOXEL_CONFIG_H_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <istream>
#include <ostream>
#include <string>

#include <wavemap/core/map/cell_types/cell_data_traits.h>
#include <wavemap/core/map/cell_types/voxel_data.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/core/map/hashed_wavelet_octree_block.h>
#include <wavemap/io/stream_conversions.h>
#include <wavemap/layered/layered_map_schema.h>

struct Rgb {
  float r = 0.f;
  float g = 0.f;
  float b = 0.f;

  bool operator==(const Rgb& other) const {
    return r == other.r && g == other.g && b == other.b;
  }
};

inline Rgb operator+(const Rgb& lhs, const Rgb& rhs) {
  return {lhs.r + rhs.r, lhs.g + rhs.g, lhs.b + rhs.b};
}

inline Rgb operator-(const Rgb& lhs, const Rgb& rhs) {
  return {lhs.r - rhs.r, lhs.g - rhs.g, lhs.b - rhs.b};
}

inline Rgb operator*(float factor, const Rgb& color) {
  return {factor * color.r, factor * color.g, factor * color.b};
}

inline Rgb operator*(const Rgb& color, float factor) {
  return factor * color;
}

inline Rgb clampRgb(const Rgb& color, const Rgb& min, const Rgb& max) {
  return {std::clamp(color.r, min.r, max.r),
          std::clamp(color.g, min.g, max.g),
          std::clamp(color.b, min.b, max.b)};
}

inline float maxAbs(const Rgb& color) {
  return std::max({std::abs(color.r), std::abs(color.g), std::abs(color.b)});
}

inline float sumAbsDiff(const Rgb& lhs, const Rgb& rhs) {
  return std::abs(lhs.r - rhs.r) + std::abs(lhs.g - rhs.g) +
         std::abs(lhs.b - rhs.b);
}

struct ContinuousLayers {
  Rgb rgb{};
  float traversability = 0.f;

  bool operator==(const ContinuousLayers& other) const {
    return rgb == other.rgb && traversability == other.traversability;
  }
};

struct ContinuousLayersPolicy {
  static ContinuousLayers add(const ContinuousLayers& lhs,
                              const ContinuousLayers& rhs) {
    return ContinuousLayers{lhs.rgb + rhs.rgb,
                            lhs.traversability + rhs.traversability};
  }

  static ContinuousLayers subtract(const ContinuousLayers& lhs,
                                   const ContinuousLayers& rhs) {
    return ContinuousLayers{lhs.rgb - rhs.rgb,
                            lhs.traversability - rhs.traversability};
  }

  static ContinuousLayers scale(const ContinuousLayers& data,
                                wavemap::FloatingPoint factor) {
    return ContinuousLayers{factor * data.rgb, factor * data.traversability};
  }
};

namespace wavemap::layered {
template <>
struct ContinuousLayerSchemaTraits<::ContinuousLayers> {
  static std::vector<LayerSchemaEntry> layers() {
    return {{"color", "float32_rgb"}, {"traversability", "float32"}};
  }
};
}  // namespace wavemap::layered

namespace wavemap {
template <>
struct CellDataTraits<::ContinuousLayers> {
  struct ThresholdConfig {
    Rgb rgb_min{};
    Rgb rgb_max{1.f, 1.f, 1.f};
    FloatingPoint traversability_min = 0.f;
    FloatingPoint traversability_max = 1.f;
  };

  struct PruningConfig {
    FieldPruningConfig rgb{};
    FieldPruningConfig traversability{};
    FloatingPoint combined_threshold = 1.f;
  };

  static void threshold(ContinuousLayers& layers,
                        const ThresholdConfig& config) {
    layers.rgb = clampRgb(layers.rgb, config.rgb_min, config.rgb_max);
    layers.traversability = std::clamp(layers.traversability,
                                       config.traversability_min,
                                       config.traversability_max);
  }

  static FloatingPoint pruningScore(const ContinuousLayers& layers,
                                    const PruningConfig& config) {
    return weightedPruningScore(maxAbs(layers.rgb), config.rgb) +
           weightedPruningScore(std::abs(layers.traversability),
                                config.traversability);
  }

  static bool isNonzero(const ContinuousLayers& layers,
                        const PruningConfig& config) {
    return config.combined_threshold < pruningScore(layers, config);
  }
};
}  // namespace wavemap

using LayeredVoxel = wavemap::VoxelData<ContinuousLayers, ContinuousLayersPolicy>;
using LayeredBlock = wavemap::HashedWaveletOctreeBlockT<LayeredVoxel>;
using ContinuousWaveletMap = wavemap::HashedWaveletOctreeT<LayeredVoxel>;
using LayeredPruningConfig = ContinuousWaveletMap::PruningConfig;

// Pruning uses a normalized weighted score over all wavelet detail
// coefficients: score = sum(weight * abs(detail) / scale). A node is kept
// when score > combined_threshold. Set a field weight to 0 to ignore it.
inline LayeredPruningConfig makeLayeredPruningConfig(
    float occupancy_weight, float rgb_weight, float traversability_weight,
    float scale = 1e-3f, float combined_threshold = 1.f) {
  LayeredPruningConfig config;
  config.occupancy.scale = scale;
  config.occupancy.weight = occupancy_weight;
  config.data.rgb.scale = scale;
  config.data.rgb.weight = rgb_weight;
  config.data.traversability.scale = scale;
  config.data.traversability.weight = traversability_weight;
  config.combined_threshold = combined_threshold;
  config.data.combined_threshold = combined_threshold;
  return config;
}

inline LayeredPruningConfig makeLayeredPruningConfigWithScales(
    float occupancy_weight, float rgb_weight, float traversability_weight,
    float occupancy_scale, float rgb_scale, float traversability_scale,
    float combined_threshold = 1.f) {
  LayeredPruningConfig config;
  config.occupancy.scale = occupancy_scale;
  config.occupancy.weight = occupancy_weight;
  config.data.rgb.scale = rgb_scale;
  config.data.rgb.weight = rgb_weight;
  config.data.traversability.scale = traversability_scale;
  config.data.traversability.weight = traversability_weight;
  config.combined_threshold = combined_threshold;
  config.data.combined_threshold = combined_threshold;
  return config;
}

inline LayeredPruningConfig makeOccupancyOnlyPruningConfig(
    float scale = 1e-3f, float combined_threshold = 1.f) {
  return makeLayeredPruningConfig(1.f, 0.f, 0.f, scale, combined_threshold);
}

inline LayeredPruningConfig makeEqualLayerPruningConfig(
    float scale = 1e-3f, float combined_threshold = 1.f) {
  return makeLayeredPruningConfig(1.f, 1.f, 1.f, scale, combined_threshold);
}

struct LayeredVoxelSerializer {
  static void write(std::ostream& ostream, const LayeredVoxel& voxel) {
    wavemap::io::StreamableCellData<wavemap::FloatingPoint>::write(ostream, voxel.occupancy);
    wavemap::io::StreamableCellData<wavemap::FloatingPoint>::write(ostream, voxel.data.rgb.r);
    wavemap::io::StreamableCellData<wavemap::FloatingPoint>::write(ostream, voxel.data.rgb.g);
    wavemap::io::StreamableCellData<wavemap::FloatingPoint>::write(ostream, voxel.data.rgb.b);
    wavemap::io::StreamableCellData<wavemap::FloatingPoint>::write(ostream, voxel.data.traversability);
  }

  static LayeredVoxel read(std::istream& istream) {
    LayeredVoxel voxel;
    voxel.occupancy = wavemap::io::StreamableCellData<wavemap::FloatingPoint>::read(istream);
    voxel.data.rgb.r = wavemap::io::StreamableCellData<wavemap::FloatingPoint>::read(istream);
    voxel.data.rgb.g = wavemap::io::StreamableCellData<wavemap::FloatingPoint>::read(istream);
    voxel.data.rgb.b = wavemap::io::StreamableCellData<wavemap::FloatingPoint>::read(istream);
    voxel.data.traversability = wavemap::io::StreamableCellData<wavemap::FloatingPoint>::read(istream);
    return voxel;
  }
};

inline Rgb rgb(float r, float g, float b) {
  return Rgb{r, g, b};
}

inline LayeredVoxel makeLayeredVoxel(wavemap::FloatingPoint occupancy,
                                     const Rgb& rgb,
                                     float traversability) {
  return LayeredVoxel(occupancy, ContinuousLayers{rgb, traversability});
}

inline void printVoxel(const std::string& label, const LayeredVoxel& voxel) {
  std::cout << label << ": occ=" << voxel.occupancy
            << " trav=" << voxel.data.traversability << " rgb=("
            << voxel.data.rgb.r << ", " << voxel.data.rgb.g << ", "
            << voxel.data.rgb.b << ")\n";
}

#endif  // WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_LAYERED_VOXEL_CONFIG_H_
