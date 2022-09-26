#ifndef WAVEMAP_RVIZ_PLUGIN_COMMON_H_
#define WAVEMAP_RVIZ_PLUGIN_COMMON_H_

#include <wavemap_3d/data_structure/volumetric_octree.h>
#include <wavemap_common/data_structure/volumetric/cell_types/scalar_cell.h>

namespace wavemap::rviz_plugin {
using Octree = VolumetricOctree<UnboundedScalarCell>;
}

#endif  // WAVEMAP_RVIZ_PLUGIN_COMMON_H_
