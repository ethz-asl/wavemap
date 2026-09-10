#ifndef WAVEMAP_LAYERED_CONTINUOUS_LAYER_UPDATER_H_
#define WAVEMAP_LAYERED_CONTINUOUS_LAYER_UPDATER_H_

#include <utility>

#include <wavemap/core/common.h>

namespace wavemap::layered {

// Mutates only the additional continuous data and applies the resulting delta
// with zero occupancy. This preserves occupancy and every field the mutator
// leaves unchanged while still updating the wavelet representation correctly.
template <typename ContinuousMapT, typename MutatorT>
void updateContinuousLayersAtVoxel(
    ContinuousMapT& map, const wavemap::Index3D& index, MutatorT&& mutator) {
  using Voxel = typename ContinuousMapT::CellDataType;
  using ContinuousPolicy = typename Voxel::AdditionalDataPolicy;

  const Voxel current_voxel = map.getVoxelValue(index);
  auto updated_data = current_voxel.data;
  std::forward<MutatorT>(mutator)(updated_data);

  const auto data_update =
      ContinuousPolicy::subtract(updated_data, current_voxel.data);
  map.addToVoxelValue(index, Voxel(0.f, data_update));
}

template <typename ContinuousMapT>
void setContinuousLayersAtVoxel(
    ContinuousMapT& map, const wavemap::Index3D& index,
    const typename ContinuousMapT::CellDataType::AdditionalData& data) {
  updateContinuousLayersAtVoxel(
      map, index, [&data](auto& current_data) { current_data = data; });
}

}  // namespace wavemap::layered

#endif  // WAVEMAP_LAYERED_CONTINUOUS_LAYER_UPDATER_H_
