#ifndef WAVEMAP_IO_STREAM_CONVERSIONS_H_
#define WAVEMAP_IO_STREAM_CONVERSIONS_H_

#include <istream>
#include <ostream>

#include <wavemap/common.h>
#include <wavemap/data_structure/volumetric/cell_types/haar_coefficients.h>
#include <wavemap/data_structure/volumetric/hashed_blocks.h>
#include <wavemap/data_structure/volumetric/hashed_chunked_wavelet_octree.h>
#include <wavemap/data_structure/volumetric/hashed_wavelet_octree.h>
#include <wavemap/data_structure/volumetric/wavelet_octree.h>

#include "wavemap_io/streamable_types.h"

namespace wavemap::io {
bool mapToStream(const VolumetricDataStructureBase& map, std::ostream& ostream);
bool streamToMap(std::istream& istream, VolumetricDataStructureBase::Ptr& map);

void mapToStream(const HashedBlocks& map, std::ostream& ostream);
bool streamToMap(std::istream& istream, HashedBlocks::Ptr& map);

void mapToStream(const WaveletOctree& map, std::ostream& ostream);
bool streamToMap(std::istream& istream, WaveletOctree::Ptr& map);

void mapToStream(const HashedWaveletOctree& map, std::ostream& ostream);
bool streamToMap(std::istream& istream, HashedWaveletOctree::Ptr& map);

void mapToStream(const HashedChunkedWaveletOctree& map, std::ostream& ostream);
}  // namespace wavemap::io

#endif  // WAVEMAP_IO_STREAM_CONVERSIONS_H_
