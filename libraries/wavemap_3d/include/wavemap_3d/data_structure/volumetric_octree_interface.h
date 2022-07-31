#ifndef WAVEMAP_3D_DATA_STRUCTURE_VOLUMETRIC_OCTREE_INTERFACE_H_
#define WAVEMAP_3D_DATA_STRUCTURE_VOLUMETRIC_OCTREE_INTERFACE_H_

#include <memory>

#include <wavemap_common/data_structure/volumetric/volumetric_ndtree_interface.h>

#include "wavemap_3d/data_structure/volumetric_data_structure_3d.h"

namespace wavemap {
using VolumetricOctreeInterface = VolumetricNdtreeInterface<3>;
}  // namespace wavemap

#endif  // WAVEMAP_3D_DATA_STRUCTURE_VOLUMETRIC_OCTREE_INTERFACE_H_
