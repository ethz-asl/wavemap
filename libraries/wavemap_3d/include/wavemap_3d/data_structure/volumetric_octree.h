#ifndef WAVEMAP_3D_DATA_STRUCTURE_VOLUMETRIC_OCTREE_H_
#define WAVEMAP_3D_DATA_STRUCTURE_VOLUMETRIC_OCTREE_H_

#include <wavemap/data_structure/volumetric/volumetric_ndtree.h>

#include "wavemap_3d/data_structure/volumetric_octree_interface.h"

namespace wavemap {
template <typename CellT>
using VolumetricOctree = VolumetricNdtree<CellT, 3>;
}  // namespace wavemap

#endif  // WAVEMAP_3D_DATA_STRUCTURE_VOLUMETRIC_OCTREE_H_
