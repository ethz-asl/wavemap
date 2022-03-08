#ifndef WAVEMAP_2D_UTILS_EVALUATION_UTILS_INL_H_
#define WAVEMAP_2D_UTILS_EVALUATION_UTILS_INL_H_

#include <memory>

namespace wavemap_2d::utils {
template <typename CellType, typename PredictedMap>
MapEvaluationSummary EvaluateMap(const DenseGrid<CellType>& reference_map,
                                 const PredictedMap& predicted_map,
                                 const MapEvaluationConfig& config,
                                 DenseGrid<CellType>* error_grid) {
  MapEvaluationSummary result;

  // Setup resolution conversions between the reference and predicted map
  const bool iterate_over_reference =
      (config.iterate_over != MapEvaluationConfig::Source::kPredicted);
  const FloatingPoint index_ratio =
      iterate_over_reference
          ? reference_map.getResolution() / predicted_map.getResolution()
          : predicted_map.getResolution() / reference_map.getResolution();
  auto convertIndex = [index_ratio](const Index& index) {
    return computeNearestIndexForPoint(index.cast<FloatingPoint>(),
                                       index_ratio);
  };

  // Determine the box over which we'll iterate
  const bool crop_to_reference =
      (config.crop_to != MapEvaluationConfig::Source::kPredicted);
  Index min_index = crop_to_reference ? reference_map.getMinIndex()
                                      : predicted_map.getMinIndex();
  Index max_index = crop_to_reference ? reference_map.getMaxIndex()
                                      : predicted_map.getMaxIndex();
  if (config.crop_to != config.iterate_over) {
    min_index = (min_index.cast<FloatingPoint>() / index_ratio)
                    .array()
                    .ceil()
                    .template cast<IndexElement>();
    max_index = (max_index.cast<FloatingPoint>() / index_ratio)
                    .array()
                    .floor()
                    .template cast<IndexElement>();
  }

  // Evaluate all cells in the box, at the resolution set by config.iterate_over
  for (const Index& index : Grid(min_index, max_index)) {
    const Index reference_index =
        iterate_over_reference ? index : convertIndex(index);
    OccupancyState reference_state = GetCellState(
        reference_map, reference_index, config.reference.cell_selector,
        config.reference.treat_unknown_cells_as);
    if (reference_state.isUnknown()) {
      ++result.num_cells_ignored;
      continue;
    }

    const Index predicted_index =
        iterate_over_reference ? convertIndex(index) : index;
    const OccupancyState predicted_state = GetCellState(
        predicted_map, predicted_index, config.predicted.cell_selector,
        config.predicted.treat_unknown_cells_as);
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

#endif  // WAVEMAP_2D_UTILS_EVALUATION_UTILS_INL_H_
