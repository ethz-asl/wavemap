#ifndef WAVEMAP_2D_UTILS_EVALUATION_UTILS_H_
#define WAVEMAP_2D_UTILS_EVALUATION_UTILS_H_

#include <string>

#include "wavemap_2d/datastructure/dense_grid/dense_grid.h"
#include "wavemap_2d/datastructure/occupancy_state.h"
#include "wavemap_2d/integrator/grid_iterator.h"

namespace wavemap_2d::utils {
struct CellSelector {
  enum class Categories {
    kUnknown,
    kAnyObserved,
    kFree,
    kOccupied,
    kAny
  } category;

  bool matches(OccupancyState cell_state) const {
    switch (category) {
      case Categories::kAny:
        return true;
      case Categories::kUnknown:
        return !cell_state.isUnknown();
      case Categories::kAnyObserved:
        return cell_state.isObserved();
      case Categories::kFree:
        return cell_state.isFree();
      case Categories::kOccupied:
        return cell_state.isOccupied();
    }
    return false;
  }
};

struct MapEvaluationConfig {
  enum class Source { kReference, kPredicted };
  Source iterate_over = Source::kReference;
  Source crop_to = Source::kReference;

  struct {
    CellSelector cell_selector = {CellSelector::Categories::kAnyObserved};
    OccupancyState treat_unknown_cells_as = OccupancyState::Unknown();
  } reference;
  struct {
    CellSelector cell_selector = {CellSelector::Categories::kAnyObserved};
    OccupancyState treat_unknown_cells_as = OccupancyState::Unknown();
  } predicted;
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

template <typename CellType, typename PredictedMap>
MapEvaluationSummary EvaluateMap(const DenseGrid<CellType>& reference_map,
                                 const PredictedMap& predicted_map,
                                 const MapEvaluationConfig& config,
                                 DenseGrid<CellType>* error_grid = nullptr);

template <typename Map>
OccupancyState GetCellState(const Map& map, const Index& index,
                            const CellSelector& cell_selector,
                            OccupancyState treat_unknown_cells_as);
}  // namespace wavemap_2d::utils

#include "wavemap_2d/utils/evaluation_utils_impl.h"

#endif  // WAVEMAP_2D_UTILS_EVALUATION_UTILS_H_
