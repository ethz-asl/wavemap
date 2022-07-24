#ifndef WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_POSED_RANGE_IMAGE_H_
#define WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_POSED_RANGE_IMAGE_H_

#include "wavemap_2d/integrator/scan_integrator/range_image.h"

namespace wavemap_2d {
class PosedRangeImage : public RangeImage {
 public:
  using RangeImage::RangeImage;

  PosedRangeImage(FloatingPoint min_angle, FloatingPoint max_angle,
                  Eigen::Index num_beams,
                  const PosedPointcloud<>& posed_pointcloud)
      : RangeImage(min_angle, max_angle, num_beams) {
    importPointcloud(posed_pointcloud);
  }

  void importPointcloud(const PosedPointcloud<>& posed_pointcloud) {
    RangeImage::importPointcloud(posed_pointcloud.getPointsLocal());
    setPose(posed_pointcloud.getPose());
  }

  void setPose(const Transformation& T_W_C) {
    T_W_C_ = T_W_C;
    T_C_W_ = T_W_C.inverse();
  }
  const Transformation& getPose() const { return T_W_C_; }
  const Transformation& getPoseInverse() const { return T_C_W_; }

 private:
  Transformation T_W_C_;
  Transformation T_C_W_;
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_POSED_RANGE_IMAGE_H_
