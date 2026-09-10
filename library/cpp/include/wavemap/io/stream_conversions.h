#ifndef WAVEMAP_IO_STREAM_CONVERSIONS_H_
#define WAVEMAP_IO_STREAM_CONVERSIONS_H_

#include <istream>
#include <ostream>
#include <type_traits>

#include "wavemap/core/common.h"
#include "wavemap/core/map/cell_types/haar_coefficients.h"
#include "wavemap/core/map/hashed_blocks.h"
#include "wavemap/core/map/hashed_chunked_wavelet_octree.h"
#include "wavemap/core/map/hashed_wavelet_octree.h"
#include "wavemap/core/map/wavelet_octree.h"
#include "wavemap/io/streamable_types.h"

namespace wavemap::io {
bool mapToStream(const MapBase& map, std::ostream& ostream);
bool streamToMap(std::istream& istream, MapBase::Ptr& map);

struct StreamableFloatingPoint {
  static void write(std::ostream& ostream, FloatingPoint value) {
    streamable::Float serialized_value = value;
    ostream.write(reinterpret_cast<const char*>(&serialized_value),
                  sizeof(serialized_value));
  }

  static FloatingPoint read(std::istream& istream) {
    streamable::Float serialized_value{};
    istream.read(reinterpret_cast<char*>(&serialized_value),
                 sizeof(serialized_value));
    return serialized_value;
  }
};

template <typename CellDataT, typename = void>
struct StreamableCellData;

template <>
struct StreamableCellData<FloatingPoint> : StreamableFloatingPoint {};

template <typename CellDataT, typename CellDataSerializerT>
bool mapToStream(const HashedWaveletOctreeT<CellDataT>& map,
                 std::ostream& ostream);

template <typename CellDataT, typename CellDataSerializerT>
bool streamToMap(std::istream& istream,
                 typename HashedWaveletOctreeT<CellDataT>::Ptr& map);

template <typename CellDataT, typename CellDataSerializerT>
bool mapToStream(const HashedChunkedWaveletOctreeT<CellDataT>& map,
                 std::ostream& ostream);

template <typename CellDataT, typename CellDataSerializerT>
bool streamToMap(
    std::istream& istream,
    typename HashedChunkedWaveletOctreeT<CellDataT>::Ptr& map);

bool mapToStream(const HashedBlocks& map, std::ostream& ostream);
bool streamToMap(std::istream& istream, HashedBlocks::Ptr& map);

bool mapToStream(const WaveletOctree& map, std::ostream& ostream);
bool streamToMap(std::istream& istream, WaveletOctree::Ptr& map);

bool mapToStream(const HashedWaveletOctree& map, std::ostream& ostream);
bool streamToMap(std::istream& istream, HashedWaveletOctree::Ptr& map);

bool mapToStream(const HashedChunkedWaveletOctree& map, std::ostream& ostream);
}  // namespace wavemap::io

#include "wavemap/io/impl/stream_conversions_inl.h"

#endif  // WAVEMAP_IO_STREAM_CONVERSIONS_H_
