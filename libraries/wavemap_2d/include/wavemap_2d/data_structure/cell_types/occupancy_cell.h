#ifndef WAVEMAP_2D_DATA_STRUCTURE_CELL_TYPES_OCCUPANCY_CELL_H_
#define WAVEMAP_2D_DATA_STRUCTURE_CELL_TYPES_OCCUPANCY_CELL_H_

#include "wavemap_2d/data_structure/cell_types/scalar_cell.h"

namespace wavemap {
using UnboundedOccupancyCell = UnboundedScalarCell;
using SaturatingOccupancyCell = BoundedScalarCell<-2, 4>;
}  // namespace wavemap

#endif  // WAVEMAP_2D_DATA_STRUCTURE_CELL_TYPES_OCCUPANCY_CELL_H_
