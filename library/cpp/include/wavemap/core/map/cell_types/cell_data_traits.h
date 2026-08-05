#ifndef WAVEMAP_CORE_MAP_CELL_TYPES_CELL_DATA_TRAITS_H_
#define WAVEMAP_CORE_MAP_CELL_TYPES_CELL_DATA_TRAITS_H_

#include <cmath>
#include <limits>
#include <type_traits>

#include "wavemap/core/common.h"
#include "wavemap/core/map/cell_types/voxel_data.h"

namespace wavemap {
namespace detail {
template <typename>
inline constexpr bool kAlwaysFalse = false;
}  // namespace detail

struct EmptyCellDataConfig {};

struct FieldPruningConfig {
  FloatingPoint scale = 1e-3f;
  FloatingPoint weight = 1.f;
};

inline FloatingPoint weightedPruningScore(
    FloatingPoint magnitude, const FieldPruningConfig& config) {
  if (config.weight <= 0.f || magnitude <= 0.f) {
    return 0.f;
  }
  if (config.scale <= 0.f) {
    return std::numeric_limits<FloatingPoint>::infinity();
  }
  return config.weight * magnitude / config.scale;
}

template <typename CellDataT>
struct CellDataTraits {
  using ThresholdConfig = EmptyCellDataConfig;
  using PruningConfig = EmptyCellDataConfig;

  static void threshold(CellDataT&, const ThresholdConfig&) {
    static_assert(detail::kAlwaysFalse<CellDataT>,
                  "CellDataTraits<T>::threshold must be specialized for this "
                  "custom cell data type.");
  }

  static FloatingPoint pruningScore(const CellDataT&, const PruningConfig&) {
    static_assert(detail::kAlwaysFalse<CellDataT>,
                  "CellDataTraits<T>::pruningScore must be specialized for "
                  "this custom cell data type.");
    return 0.f;
  }

  static bool isNonzero(const CellDataT& value, const PruningConfig& config) {
    return 1.f < pruningScore(value, config);
  }
};

template <>
struct CellDataTraits<FloatingPoint> {
  using ThresholdConfig = EmptyCellDataConfig;
  using PruningConfig = FieldPruningConfig;

  static void threshold(FloatingPoint&, const ThresholdConfig&) {}

  static FloatingPoint pruningScore(FloatingPoint value,
                                    const PruningConfig& config) {
    return weightedPruningScore(std::abs(value), config);
  }

  static bool isNonzero(FloatingPoint value, const PruningConfig& config) {
    return 1.f < pruningScore(value, config);
  }
};

template <typename AdditionalDataT, typename AdditionalDataPolicyT>
struct CellDataTraits<VoxelData<AdditionalDataT, AdditionalDataPolicyT>> {
  using AdditionalTraits = CellDataTraits<AdditionalDataT>;

  struct ThresholdConfig {
    typename AdditionalTraits::ThresholdConfig data{};
  };

  struct PruningConfig {
    FieldPruningConfig occupancy{};
    typename AdditionalTraits::PruningConfig data{};
    FloatingPoint combined_threshold = 1.f;
  };

  static void threshold(
      VoxelData<AdditionalDataT, AdditionalDataPolicyT>& voxel,
      const ThresholdConfig& config) {
    AdditionalTraits::threshold(voxel.data, config.data);
  }

  static FloatingPoint pruningScore(
      const VoxelData<AdditionalDataT, AdditionalDataPolicyT>& voxel,
      const PruningConfig& config) {
    return weightedPruningScore(std::abs(voxel.occupancy), config.occupancy) +
           AdditionalTraits::pruningScore(voxel.data, config.data);
  }

  static bool isNonzero(
      const VoxelData<AdditionalDataT, AdditionalDataPolicyT>& voxel,
      const PruningConfig& config) {
    return config.combined_threshold < pruningScore(voxel, config);
  }
};

template <typename AdditionalDataPolicyT>
struct CellDataTraits<VoxelData<void, AdditionalDataPolicyT>> {
  using ThresholdConfig = EmptyCellDataConfig;

  struct PruningConfig {
    FieldPruningConfig occupancy{};
    FloatingPoint combined_threshold = 1.f;
  };

  static void threshold(VoxelData<void, AdditionalDataPolicyT>&,
                        const ThresholdConfig&) {}

  static FloatingPoint pruningScore(
      const VoxelData<void, AdditionalDataPolicyT>& voxel,
      const PruningConfig& config) {
    return weightedPruningScore(std::abs(voxel.occupancy), config.occupancy);
  }

  static bool isNonzero(const VoxelData<void, AdditionalDataPolicyT>& voxel,
                        const PruningConfig& config) {
    return config.combined_threshold < pruningScore(voxel, config);
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_CORE_MAP_CELL_TYPES_CELL_DATA_TRAITS_H_
