#ifndef WAVEMAP_3D_DATA_STRUCTURE_WAVELET_OCTREE_H_
#define WAVEMAP_3D_DATA_STRUCTURE_WAVELET_OCTREE_H_

#include <wavemap_common/data_structure/volumetric/wavelet_ndtree.h>

#include "wavemap_3d/data_structure/wavelet_octree_interface.h"

namespace wavemap {
template <typename CellT>
using WaveletOctree = WaveletNdtree<CellT, 3>;
}

#endif  // WAVEMAP_3D_DATA_STRUCTURE_WAVELET_OCTREE_H_
