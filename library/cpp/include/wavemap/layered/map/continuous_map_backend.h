#ifndef WAVEMAP_LAYERED_MAP_CONTINUOUS_MAP_BACKEND_H_
#define WAVEMAP_LAYERED_MAP_CONTINUOUS_MAP_BACKEND_H_

#include <wavemap/core/map/hashed_chunked_wavelet_octree.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>

namespace wavemap::layered {

struct HashedWaveletOctreeBackend {
  template <typename CellDataT>
  using Map = wavemap::HashedWaveletOctreeT<CellDataT>;
};

struct HashedChunkedWaveletOctreeBackend {
  template <typename CellDataT>
  using Map = wavemap::HashedChunkedWaveletOctreeT<CellDataT>;
};

}  // namespace wavemap::layered

#endif  // WAVEMAP_LAYERED_MAP_CONTINUOUS_MAP_BACKEND_H_
