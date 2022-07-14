#ifndef WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_CELL_TYPES_OCCUPANCY_CELL_H_
#define WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_CELL_TYPES_OCCUPANCY_CELL_H_

#include "wavemap_common/data_structure/volumetric_cell_types/scalar_cell.h"

namespace wavemap {
using UnboundedOccupancyCell = UnboundedScalarCell;
using SaturatingOccupancyCell = BoundedScalarCell<-2, 4>;
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_DATA_STRUCTURE_VOLUMETRIC_CELL_TYPES_OCCUPANCY_CELL_H_
