#ifndef WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_FIXED_LOGODDS_MODEL_H_
#define WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_FIXED_LOGODDS_MODEL_H_

#include "wavemap_2d/common.h"
#include "wavemap_2d/indexing/index.h"
#include "wavemap_2d/integrator/measurement_model/measurement_model.h"
#include "wavemap_2d/iterator/ray_iterator.h"

namespace wavemap_2d {
class FixedLogOddsModel : public MeasurementModel {
 public:
  static constexpr FloatingPoint kLogOddsOccupied = 0.85f;
  static constexpr FloatingPoint kLogOddsFree = -0.4f;

  explicit FixedLogOddsModel(FloatingPoint resolution)
      : MeasurementModel(resolution) {}

  Index getBottomLeftUpdateIndex() const override;
  Index getTopRightUpdateIndex() const override;

  // NOTE: This method assumes queried indices always lies on the ray's line
  //       segment.
  FloatingPoint computeUpdateAt(const Index& index) const override {
    if (index == end_point_index_) {
      return kLogOddsOccupied;
    } else {
      return kLogOddsFree;
    }
  }
  void updateMap(DataStructureBase& map) const override;

 protected:
  Index end_point_index_;

  void updateCachedVariablesDerived() override {
    end_point_index_ =
        computeNearestIndexForPoint(W_end_point_, resolution_inv_);
  }
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_INTEGRATOR_MEASUREMENT_MODEL_FIXED_LOGODDS_MODEL_H_
