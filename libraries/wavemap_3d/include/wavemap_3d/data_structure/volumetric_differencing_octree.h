#ifndef WAVEMAP_3D_DATA_STRUCTURE_VOLUMETRIC_DIFFERENCING_OCTREE_H_
#define WAVEMAP_3D_DATA_STRUCTURE_VOLUMETRIC_DIFFERENCING_OCTREE_H_

#include <wavemap_common/data_structure/volumetric/volumetric_differencing_ndtree.h>

#include "wavemap_3d/data_structure/volumetric_octree_interface.h"

namespace wavemap {
template <typename CellT>
using VolumetricDifferencingOctree = VolumetricDifferencingNdtree<CellT, 3>;
}

#endif  // WAVEMAP_3D_DATA_STRUCTURE_VOLUMETRIC_DIFFERENCING_OCTREE_H_
