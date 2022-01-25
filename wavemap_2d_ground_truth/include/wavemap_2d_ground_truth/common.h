#ifndef WAVEMAP_2D_GROUND_TRUTH_COMMON_H_
#define WAVEMAP_2D_GROUND_TRUTH_COMMON_H_

#include <Eigen/Eigen>
#include <glog/logging.h>
#include <kindr/minimal/quat-transformation.h>
#include <wavemap_2d/common.h>

namespace wavemap_2d::ground_truth {
using Transformation2D = Transformation;
using Point2D = Point;
using Vector2D = Point;
}  // namespace wavemap_2d::ground_truth

#endif  // WAVEMAP_2D_GROUND_TRUTH_COMMON_H_
