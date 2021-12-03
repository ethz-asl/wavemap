#ifndef WAVEMAP_2D_INTEGRATOR_FIXED_LOGODDS_MODEL_H_
#define WAVEMAP_2D_INTEGRATOR_FIXED_LOGODDS_MODEL_H_

#include "wavemap_2d/common.h"
#include "wavemap_2d/integrator/measurement_model_base.h"
#include "wavemap_2d/integrator/ray_iterator.h"

namespace wavemap_2d {
class FixedLogOddsModel : public MeasurementModelBase {
 public:
  static constexpr FloatingPoint kLogOddsOccupied = 0.85f;
  static constexpr FloatingPoint kLogOddsFree = -0.4f;

  explicit FixedLogOddsModel(FloatingPoint resolution)
      : MeasurementModelBase(resolution) {}

  Index getBottomLeftUpdateIndex() override;
  Index getTopRightUpdateIndex() override;

  // NOTE: This method assumes queried indices always lies on the ray's line
  //       segment.
  FloatingPoint computeUpdateAt(const Index& index) override {
    if (index == end_point_index) {
      return kLogOddsOccupied;
    } else {
      return kLogOddsFree;
    }
  }
  void updateMap(DataStructureBase& map) override;

 protected:
  Index end_point_index;
  void updateCachedVariablesDerived() override {
    const Point end_point_scaled = W_end_point_ * resolution_inv_;
    end_point_index = end_point_scaled.array().round().cast<IndexElement>();
  }
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_INTEGRATOR_FIXED_LOGODDS_MODEL_H_
