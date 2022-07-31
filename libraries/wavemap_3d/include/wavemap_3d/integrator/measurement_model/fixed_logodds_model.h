#ifndef WAVEMAP_3D_INTEGRATOR_MEASUREMENT_MODEL_FIXED_LOGODDS_MODEL_H_
#define WAVEMAP_3D_INTEGRATOR_MEASUREMENT_MODEL_FIXED_LOGODDS_MODEL_H_

#include <wavemap_common/common.h>
#include <wavemap_common/indexing/index_conversions.h>

#include "wavemap_3d/integrator/measurement_model/measurement_model.h"

namespace wavemap {
class FixedLogOddsModel : public MeasurementModel {
 public:
  static constexpr FloatingPoint kLogOddsOccupied = 0.85f;
  static constexpr FloatingPoint kLogOddsFree = -0.4f;

  // Use the base class' constructor
  using MeasurementModel::MeasurementModel;

  Index3D getBottomLeftUpdateIndex() const override;
  Index3D getTopRightUpdateIndex() const override;

  // NOTE: This method assumes queried indices always lies on the ray's line
  //       segment.
  FloatingPoint computeUpdateAt(const Index3D& index) const override {
    if (index == end_point_index_) {
      return kLogOddsOccupied;
    } else {
      return kLogOddsFree;
    }
  }

 private:
  Index3D end_point_index_;

  void updateCachedVariablesDerived() override {
    end_point_index_ =
        convert::pointToNearestIndex(W_end_point_, min_cell_width_inv_);
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_3D_INTEGRATOR_MEASUREMENT_MODEL_FIXED_LOGODDS_MODEL_H_