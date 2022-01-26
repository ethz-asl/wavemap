#ifndef WAVEMAP_2D_UTILS_EVALUATION_UTILS_H_
#define WAVEMAP_2D_UTILS_EVALUATION_UTILS_H_

#include <string>

#include "wavemap_2d/datastructure/dense_grid/dense_grid.h"
#include "wavemap_2d/integrator/grid_iterator.h"

namespace wavemap_2d::utils {
enum class CellSource { kReference, kTest, kBoth };
enum class CellState { kUnknown, kFree, kOccupied, kAny };
enum class UnknownCellHandling {
  kAssumeFree,
  kAssumeOccupied,
  kAlwaysFalse,
  kIgnore
};
enum class CellEvaluationResult {
  kIgnore,
  kTruePositive,
  kTrueNegative,
  kFalsePositive,
  kFalseNegative
};

struct MapEvaluationSummary {
  bool is_valid = true;
  size_t num_true_positive = 0u;
  size_t num_true_negative = 0u;
  size_t num_false_positive = 0u;
  size_t num_false_negative = 0u;
  size_t num_cells_ignored = 0u;

  size_t num_cells_considered() const {
    return num_true_positive + num_true_negative + num_false_positive +
           num_false_negative;
  }
  size_t num_cells_evaluated() const {
    return num_cells_considered() + num_cells_ignored;
  }
  FloatingPoint precision() const;
  FloatingPoint recall() const;
  FloatingPoint f_1_score() const;
  std::string toString() const;
};

CellState CellStateFromValue(FloatingPoint cell_value) {
  if (std::abs(cell_value) < kEpsilon) {
    return CellState::kUnknown;
  } else if (cell_value < 0.f) {
    return CellState::kFree;
  } else {
    return CellState::kOccupied;
  }
}

struct EvaluationCellSelector {
  CellSource iterate_over_known_cells_from = CellSource::kReference;
  CellState reference_cells = CellState::kAny;
  CellState test_cells = CellState::kAny;

  static bool matches(CellState reference_state, CellState test_state) {
    if (reference_state == CellState::kAny) {
      return true;
    } else {
      return reference_state == test_state;
    }
  }
};

CellEvaluationResult EvaluateCell(
    CellState reference_state, CellState test_state,
    UnknownCellHandling unknown_test_cell_handling);

template <typename CellType, typename TestMap>
MapEvaluationSummary EvaluateMap(
    const DenseGrid<CellType>& map_reference, const TestMap& map_to_test,
    const EvaluationCellSelector& evaluation_cell_selector,
    UnknownCellHandling unknown_test_cell_handling, bool visualize = false);
}  // namespace wavemap_2d::utils

#include "wavemap_2d/utils/evaluation_utils_impl.h"

#endif  // WAVEMAP_2D_UTILS_EVALUATION_UTILS_H_
