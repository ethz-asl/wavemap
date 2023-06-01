#ifndef WAVEMAP_3D_DATA_STRUCTURE_HASHED_BLOCKS_3D_H_
#define WAVEMAP_3D_DATA_STRUCTURE_HASHED_BLOCKS_3D_H_

#include <wavemap_common/data_structure/volumetric/hashed_blocks.h>

#include "wavemap_3d/data_structure/volumetric_data_structure_3d.h"

namespace wavemap {
template <typename CellT>
using HashedBlocks3D = HashedBlocks<CellT, 3>;
}  // namespace wavemap

#endif  // WAVEMAP_3D_DATA_STRUCTURE_HASHED_BLOCKS_3D_H_
