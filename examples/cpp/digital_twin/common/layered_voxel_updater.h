#ifndef WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_LAYERED_VOXEL_UPDATER_H_
#define WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_LAYERED_VOXEL_UPDATER_H_

#include <vector>

#include <wavemap/core/indexing/index_conversions.h>

#include "layered_voxel_config.h"

struct ContinuousLayerObservation {
  wavemap::Index3D index;
  ContinuousLayers data{};
};

inline void setContinuousLayersAtVoxel(
    ContinuousWaveletMap& map, const wavemap::Index3D& index,
    const ContinuousLayers& observed_data) {
  const LayeredVoxel current_voxel = map.getVoxelValue(index);
  const ContinuousLayers data_update =
      ContinuousLayersPolicy::subtract(observed_data, current_voxel.data);
  map.addToVoxelValue(index, LayeredVoxel(0.f, data_update));
}

inline void setContinuousLayers(
    ContinuousWaveletMap& map, const std::vector<ContinuousLayerObservation>& observations) {
  for (const ContinuousLayerObservation& observation : observations) {
    setContinuousLayersAtVoxel(map, observation.index, observation.data);
  }
}

#endif  // WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_LAYERED_VOXEL_UPDATER_H_
