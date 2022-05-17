#ifndef WAVEMAP_2D_UTILS_IMPL_EVALUATION_UTILS_INL_H_
#define WAVEMAP_2D_UTILS_IMPL_EVALUATION_UTILS_INL_H_

#include <memory>

#include "wavemap_2d/indexing/index_conversions.h"

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
  const FloatingPoint reference_map_resolution = reference_map.getResolution();
  const FloatingPoint predicted_map_resolution = predicted_map.getResolution();

  // Determine the box over which we'll iterate
  const bool crop_to_reference =
      (config.crop_to != MapEvaluationConfig::Source::kPredicted);
  Index min_index;
  Index max_index;
  if (crop_to_reference) {
    if (iterate_over_reference) {
      min_index = reference_map.getMinIndex();
      max_index = reference_map.getMaxIndex();
    } else {
      min_index = convert::indexToNewResolution(reference_map.getMinIndex(),
                                                reference_map_resolution,
                                                predicted_map_resolution) +
                  Index::Ones();
      max_index = convert::indexToNewResolution(reference_map.getMaxIndex(),
                                                reference_map_resolution,
                                                predicted_map_resolution) -
                  Index::Ones();
    }
  } else {
    if (iterate_over_reference) {
      min_index = convert::indexToNewResolution(predicted_map.getMinIndex(),
                                                predicted_map_resolution,
                                                reference_map_resolution) +
                  Index::Ones();
      max_index = convert::indexToNewResolution(predicted_map.getMaxIndex(),
                                                predicted_map_resolution,
                                                reference_map_resolution) -
                  Index::Ones();
    } else {
      min_index = predicted_map.getMinIndex();
      max_index = predicted_map.getMaxIndex();
    }
  }

  // Evaluate all cells in the box, at the resolution set by config.iterate_over
  for (const Index& index : Grid(min_index, max_index)) {
    const Index reference_index =
        iterate_over_reference
            ? index
            : convert::indexToNewResolution(index, predicted_map_resolution,
                                            reference_map_resolution);
    OccupancyState reference_state = GetCellState(
        reference_map, reference_index, config.reference.cell_selector,
        config.reference.treat_unknown_cells_as);
    if (reference_state.isUnknown()) {
      ++result.num_cells_ignored;
      continue;
    }

    const Index predicted_index =
        iterate_over_reference
            ? convert::indexToNewResolution(index, reference_map_resolution,
                                            predicted_map_resolution)
            : index;
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

#endif  // WAVEMAP_2D_UTILS_IMPL_EVALUATION_UTILS_INL_H_
