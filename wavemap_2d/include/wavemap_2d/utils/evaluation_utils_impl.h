#ifndef WAVEMAP_2D_UTILS_EVALUATION_UTILS_IMPL_H_
#define WAVEMAP_2D_UTILS_EVALUATION_UTILS_IMPL_H_

#include <memory>

namespace wavemap_2d::utils {
template <typename CellType, typename PredictedMap>
MapEvaluationSummary EvaluateMap(const DenseGrid<CellType>& reference_map,
                                 const PredictedMap& predicted_map,
                                 const MapEvaluationConfig& config,
                                 DenseGrid<CellType>* error_grid) {
  MapEvaluationSummary result;

  const Index min_index = reference_map.getMinIndex();
  const Index max_index = reference_map.getMaxIndex();
  for (const Index& index : Grid(min_index, max_index)) {
    const OccupancyState reference_state =
        GetCellState(reference_map, index, config.reference_cell_selector,
                     config.reference_treat_unknown_cells_as);
    if (reference_state.isUnknown()) {
      ++result.num_cells_ignored;
      continue;
    }

    const OccupancyState predicted_state =
        GetCellState(predicted_map, index, config.predicted_cell_selector,
                     config.predicted_treat_unknown_cells_as);
    if (predicted_state.isUnknown()) {
      ++result.num_cells_ignored;
      continue;
    }

    std::optional<FloatingPoint> error_value;
    if (predicted_state.isOccupied()) {
      if (reference_state.isOccupied()) {
        ++result.num_true_positive;
        error_value = 2.f;
      } else {
        ++result.num_false_positive;
        error_value = -2.f;
      }
    } else {
      if (reference_state.isFree()) {
        ++result.num_true_negative;
        error_value = 1.f;
      } else {
        ++result.num_false_negative;
        error_value = -1.f;
      }
    }
    if (error_grid && error_value) {
      error_grid->setCellValue(index, error_value.value());
    }
  }

  return result;
}

template <typename Map>
OccupancyState GetCellState(const Map& map, const Index& index,
                            const CellSelector& cell_selector,
                            OccupancyState treat_unknown_cells_as) {
  const auto cell_value = map.getCellValue(index);
  const OccupancyState cell_state = OccupancyState::fromValue(cell_value);

  if (!cell_selector.matches(cell_state)) {
    return OccupancyState::Unknown();
  }
  if (cell_state.isUnknown()) {
    return treat_unknown_cells_as;
  }

  return cell_state;
}
}  // namespace wavemap_2d::utils

#endif  // WAVEMAP_2D_UTILS_EVALUATION_UTILS_IMPL_H_
