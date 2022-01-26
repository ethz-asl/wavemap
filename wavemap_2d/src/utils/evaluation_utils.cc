#include "wavemap_2d/utils/evaluation_utils.h"

namespace wavemap_2d::utils {
CellEvaluationResult EvaluateCell(
    CellState reference_state, CellState test_state,
    UnknownCellHandling unknown_test_cell_handling) {
  if (test_state == CellState::kUnknown) {
    switch (unknown_test_cell_handling) {
      case UnknownCellHandling::kIgnore:
        return CellEvaluationResult::kIgnore;
      case UnknownCellHandling::kAlwaysFalse:
        if (reference_state == CellState::kOccupied) {
          return CellEvaluationResult::kFalseNegative;
        } else {
          return CellEvaluationResult::kFalsePositive;
        }
      case UnknownCellHandling::kAssumeFree:
        test_state = CellState::kFree;
        break;
      case UnknownCellHandling::kAssumeOccupied:
        test_state = CellState::kOccupied;
        break;
    }
  }

  if (reference_state == test_state) {
    if (test_state == CellState::kOccupied) {
      return CellEvaluationResult::kTruePositive;
    } else {
      return CellEvaluationResult::kTrueNegative;
    }
  } else {
    if (test_state == CellState::kOccupied) {
      return CellEvaluationResult::kFalsePositive;
    } else {
      return CellEvaluationResult::kFalseNegative;
    }
  }
}

FloatingPoint MapEvaluationSummary::precision() const {
  return static_cast<FloatingPoint>(num_true_positive) /
         static_cast<FloatingPoint>(num_true_positive + num_false_positive);
}

FloatingPoint MapEvaluationSummary::recall() const {
  return static_cast<FloatingPoint>(num_true_positive) /
         static_cast<FloatingPoint>(num_true_positive + num_false_negative);
}

FloatingPoint MapEvaluationSummary::f_1_score() const {
  return 2.f / (1.f / recall() + 1.f / precision());
}

std::string MapEvaluationSummary::toString() const {
  if (!is_valid) {
    return "Map evaluation failed!";
  }
  std::ostringstream summary;
  summary << "Evaluated " << num_cells_evaluated() << ", of which "
          << num_cells_considered() << " were considered and "
          << num_cells_ignored << " ignored.\n"
          << "Error distribution:\n"
          << "-- true positives: " << num_true_positive << "\n"
          << "-- true negatives: " << num_true_negative << "\n"
          << "-- false positives: " << num_false_positive << "\n"
          << "-- false negatives: " << num_false_negative << "\n"
          << "Precision: " << precision() << "\n"
          << "Recall: " << recall() << "\n"
          << "F1 score: " << f_1_score() << "\n";
  return summary.str();
}
}  // namespace wavemap_2d::utils
