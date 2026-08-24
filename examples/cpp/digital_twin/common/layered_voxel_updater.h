#ifndef WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_LAYERED_VOXEL_UPDATER_H_
#define WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_LAYERED_VOXEL_UPDATER_H_

#include <vector>

#include <wavemap/core/indexing/index_conversions.h>
#include <wavemap/layered/continuous_layer_updater.h>

#include "layered_voxel_config.h"

struct ContinuousLayerObservation {
  wavemap::Index3D index;
  ContinuousLayers data{};
};

inline void setContinuousLayersAtVoxel(
    ContinuousWaveletMap& map, const wavemap::Index3D& index,
    const ContinuousLayers& observed_data) {
  wavemap::layered::setContinuousLayersAtVoxel(map, index, observed_data);
}

inline void setContinuousLayers(
    ContinuousWaveletMap& map, const std::vector<ContinuousLayerObservation>& observations) {
  for (const ContinuousLayerObservation& observation : observations) {
    setContinuousLayersAtVoxel(map, observation.index, observation.data);
  }
}

#endif  // WAVEMAP_EXAMPLES_CPP_DIGITAL_TWIN_LAYERED_VOXEL_UPDATER_H_
