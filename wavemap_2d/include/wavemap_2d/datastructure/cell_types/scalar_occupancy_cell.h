#ifndef WAVEMAP_2D_DATASTRUCTURE_CELL_TYPES_SCALAR_OCCUPANCY_CELL_H_
#define WAVEMAP_2D_DATASTRUCTURE_CELL_TYPES_SCALAR_OCCUPANCY_CELL_H_

#include "wavemap_2d/datastructure/cell_types/scalar_cell.h"

namespace wavemap_2d {
using UnboundedOccupancyCell = UnboundedScalarCell;
using SaturatingOccupancyCell = BoundedScalarCell<-2, 4>;
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATASTRUCTURE_CELL_TYPES_SCALAR_OCCUPANCY_CELL_H_
