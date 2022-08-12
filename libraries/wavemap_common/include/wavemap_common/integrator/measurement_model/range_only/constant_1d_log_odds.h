#ifndef WAVEMAP_COMMON_INTEGRATOR_MEASUREMENT_MODEL_RANGE_ONLY_CONSTANT_1D_LOG_ODDS_H_
#define WAVEMAP_COMMON_INTEGRATOR_MEASUREMENT_MODEL_RANGE_ONLY_CONSTANT_1D_LOG_ODDS_H_

#include <wavemap_common/common.h>
#include <wavemap_common/indexing/index_conversions.h>
#include <wavemap_common/integrator/measurement_model/measurement_model_base.h>

namespace wavemap {
template <int dim>
class Constant1DLogOdds : public MeasurementModelBase<dim> {
 public:
  static constexpr FloatingPoint kLogOddsOccupied = 0.85f;
  static constexpr FloatingPoint kLogOddsFree = -0.4f;

  // Use the base class' constructor
  using MeasurementModelBase<dim>::MeasurementModelBase;

  Index<dim> getBottomLeftUpdateIndex() const override {
    const Point<dim> bottom_left_point =
        Base::W_start_point_.cwiseMin(Base::W_end_point_);
    return convert::pointToFloorIndex(bottom_left_point,
                                      Base::min_cell_width_inv_);
  }
  Index<dim> getTopRightUpdateIndex() const override {
    const Point<dim> top_right_point =
        Base::W_start_point_.cwiseMax(Base::W_end_point_);
    return convert::pointToCeilIndex(top_right_point,
                                     Base::min_cell_width_inv_);
  }

  // NOTE: This method assumes queried indices always lies on the ray's line
  //       segment.
  FloatingPoint computeUpdate(const Index<dim>& index) const {
    if (index == end_point_index_) {
      return kLogOddsOccupied;
    } else {
      return kLogOddsFree;
    }
  }

 private:
  using Base = MeasurementModelBase<dim>;
  Index<dim> end_point_index_;

  void updateCachedVariablesDerived() override {
    end_point_index_ = convert::pointToNearestIndex(Base::W_end_point_,
                                                    Base::min_cell_width_inv_);
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_INTEGRATOR_MEASUREMENT_MODEL_RANGE_ONLY_CONSTANT_1D_LOG_ODDS_H_
