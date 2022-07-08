#ifndef WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_POSED_RANGE_IMAGE_H_
#define WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_POSED_RANGE_IMAGE_H_

#include "wavemap_2d/integrator/scan_integrator/range_image.h"

namespace wavemap {
class PosedRangeImage : public RangeImage {
 public:
  using RangeImage::RangeImage;

  PosedRangeImage(
      FloatingPoint min_angle, FloatingPoint max_angle, Eigen::Index num_beams,
      const PosedPointcloud<Point2D, Transformation2D>& posed_pointcloud)
      : RangeImage(min_angle, max_angle, num_beams) {
    importPointcloud(posed_pointcloud);
  }

  void importPointcloud(
      const PosedPointcloud<Point2D, Transformation2D>& posed_pointcloud) {
    RangeImage::importPointcloud(posed_pointcloud.getPointsLocal());
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
