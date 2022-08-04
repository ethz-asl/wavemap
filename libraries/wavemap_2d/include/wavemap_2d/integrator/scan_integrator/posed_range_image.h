#ifndef WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_POSED_RANGE_IMAGE_H_
#define WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_POSED_RANGE_IMAGE_H_

#include "wavemap_2d/integrator/scan_integrator/range_image_1d.h"

namespace wavemap {
class PosedRangeImage : public RangeImage1D {
 public:
  using RangeImage1D::RangeImage1D;

  PosedRangeImage(
      const PosedPointcloud<Point2D, Transformation2D>& posed_pointcloud,
      const CircularProjector& circular_projector)
      : RangeImage1D(posed_pointcloud.getPointsLocal(), circular_projector) {
    setPose(posed_pointcloud.getPose());
  }

  void importPointcloud(
      const PosedPointcloud<Point2D, Transformation2D>& posed_pointcloud,
      const CircularProjector& circular_projector) {
    RangeImage1D::importPointcloud(posed_pointcloud.getPointsLocal(),
                                   circular_projector);
    setPose(posed_pointcloud.getPose());
  }

  void setPose(const Transformation2D& T_W_C) {
    T_W_C_ = T_W_C;
    T_C_W_ = T_W_C.inverse();
  }
  const Transformation2D& getPose() const { return T_W_C_; }
  const Transformation2D& getPoseInverse() const { return T_C_W_; }

 private:
  Transformation2D T_W_C_;
  Transformation2D T_C_W_;
};
}  // namespace wavemap

#endif  // WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_POSED_RANGE_IMAGE_H_
