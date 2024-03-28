#ifndef WAVEMAP_IO_STREAM_CONVERSIONS_H_
#define WAVEMAP_IO_STREAM_CONVERSIONS_H_

#include <istream>
#include <ostream>

#include "wavemap/common.h"
#include "wavemap/io/streamable_types.h"
#include "wavemap/map/cell_types/haar_coefficients.h"
#include "wavemap/map/hashed_blocks.h"
#include "wavemap/map/hashed_chunked_wavelet_octree.h"
#include "wavemap/map/hashed_wavelet_octree.h"
#include "wavemap/map/wavelet_octree.h"

namespace wavemap::io {
bool mapToStream(const MapBase& map, std::ostream& ostream);
bool streamToMap(std::istream& istream, MapBase::Ptr& map);

void mapToStream(const HashedBlocks& map, std::ostream& ostream);
bool streamToMap(std::istream& istream, HashedBlocks::Ptr& map);

void mapToStream(const WaveletOctree& map, std::ostream& ostream);
bool streamToMap(std::istream& istream, WaveletOctree::Ptr& map);

void mapToStream(const HashedWaveletOctree& map, std::ostream& ostream);
bool streamToMap(std::istream& istream, HashedWaveletOctree::Ptr& map);

void mapToStream(const HashedChunkedWaveletOctree& map, std::ostream& ostream);
}  // namespace wavemap::io

#endif  // WAVEMAP_IO_STREAM_CONVERSIONS_H_
