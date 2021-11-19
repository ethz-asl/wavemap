#ifndef WAVEMAP_2D_COMMON_H_
#define WAVEMAP_2D_COMMON_H_

#include <vector>

#include <Eigen/Eigen>
#include <kindr/minimal/transform-2d.h>

namespace wavemap_2d {
using FloatingPoint = float;
constexpr FloatingPoint kEpsilon = 1e-6;

using Transformation = kindr::minimal::Transformation2DTemplate<FloatingPoint>;
using Point = Transformation::Position;
using Translation = Transformation::Position;
using Rotation = Transformation::Rotation;

using IndexElement = int;
using Index = Eigen::Matrix<IndexElement, 2, 1>;
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_COMMON_H_
