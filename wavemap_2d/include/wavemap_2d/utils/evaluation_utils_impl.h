#ifndef WAVEMAP_2D_UTILS_EVALUATION_UTILS_IMPL_H_
#define WAVEMAP_2D_UTILS_EVALUATION_UTILS_IMPL_H_

#include <memory>

namespace wavemap_2d::utils {
template <typename CellType, typename TestMap>
MapEvaluationSummary EvaluateMap(
    const DenseGrid<CellType>& map_reference, const TestMap& map_to_test,
    const EvaluationCellSelector& evaluation_cell_selector,
    UnknownCellHandling unknown_test_cell_handling, bool visualize) {
  std::unique_ptr<DenseGrid<CellType>> error_map;
  if (visualize) {
    error_map =
        std::make_unique<DenseGrid<CellType>>(map_reference.getResolution());
  }

  if (evaluation_cell_selector.iterate_over_known_cells_from !=
      CellSource::kReference) {
    LOG(ERROR) << "Evaluations currently only support iterating over the "
                  "reference map.";
    return MapEvaluationSummary{.is_valid = false};
  }

  MapEvaluationSummary result;
  const Index min_index = map_reference.getMinIndex();
  const Index max_index = map_reference.getMaxIndex();
  Grid evaluation_grid(min_index, max_index);
  for (const Index& index : evaluation_grid) {
    auto reference_value = map_reference.getCellValue(index);
    const CellState reference_state = CellStateFromValue(reference_value);
    if (reference_state == CellState::kUnknown ||
        !EvaluationCellSelector::matches(
            evaluation_cell_selector.reference_cells, reference_state)) {
      continue;
    }

    auto test_value = map_to_test.getCellValue(index);
    const CellState test_state = CellStateFromValue(test_value);
    if (!EvaluationCellSelector::matches(evaluation_cell_selector.test_cells,
                                         test_state)) {
      ++result.num_cells_ignored;
      continue;
    }

    switch (
        EvaluateCell(reference_state, test_state, unknown_test_cell_handling)) {
      case CellEvaluationResult::kTruePositive:
        ++result.num_true_positive;
        if (visualize) {
          error_map->setCellValue(index, 2.f);
        }
        break;
      case CellEvaluationResult::kTrueNegative:
        ++result.num_true_negative;
        if (visualize) {
          error_map->setCellValue(index, 1.f);
        }
        break;
      case CellEvaluationResult::kFalsePositive:
        ++result.num_false_positive;
        if (visualize) {
          error_map->setCellValue(index, -2.f);
        }
        break;
      case CellEvaluationResult::kFalseNegative:
        ++result.num_false_negative;
        if (visualize) {
          error_map->setCellValue(index, -1.f);
        }
        break;
      default:
        ++result.num_cells_ignored;
        break;
    }
  }

  if (visualize) {
    error_map->showImage(true);
  }

  return result;
}
}  // namespace wavemap_2d::utils

#endif  // WAVEMAP_2D_UTILS_EVALUATION_UTILS_IMPL_H_
