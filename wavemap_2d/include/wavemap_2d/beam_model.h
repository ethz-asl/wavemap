#ifndef WAVEMAP_2D_BEAM_MODEL_H_
#define WAVEMAP_2D_BEAM_MODEL_H_

#include "wavemap_2d/common.h"

namespace wavemap_2d {
class BeamModel {
 public:
  BeamModel() = default;

  void setStartPoint(const Point& start_point) { start_point_ = start_point; }
  void setEndPoint(const Point& end_point) { end_point_ = end_point; }

 protected:
  Point start_point_;
  Point end_point_;
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_BEAM_MODEL_H_
