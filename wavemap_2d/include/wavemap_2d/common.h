#ifndef WAVEMAP_2D_COMMON_H_
#define WAVEMAP_2D_COMMON_H_

#include <vector>

#include <Eigen/Eigen>
#include <kindr/minimal/quat-transformation.h>
#include <kindr/minimal/transform-2d.h>

#include "wavemap_2d/constants.h"

namespace wavemap_2d {
using FloatingPoint = float;
constexpr int kMapDimension = 2;

template <typename T>
using MatrixT = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
using Matrix = MatrixT<FloatingPoint>;

using Transformation = kindr::minimal::Transformation2DTemplate<FloatingPoint>;
using Point = Transformation::Position;
using Vector = Transformation::Position;
using Rotation = Transformation::Rotation;

using Transformation3D =
    kindr::minimal::QuatTransformationTemplate<FloatingPoint>;
using Point3D = Transformation3D::Position;
using Vector3D = Transformation3D::Position;
using Rotation3D = Transformation3D::Rotation;

constexpr auto kEpsilon = constants<FloatingPoint>::kEpsilon;
constexpr auto kPi = constants<FloatingPoint>::kPi;
constexpr auto kTwoPi = constants<FloatingPoint>::kTwoPi;
constexpr auto kHalfPi = constants<FloatingPoint>::kHalfPi;
constexpr auto kQuarterPi = constants<FloatingPoint>::kQuarterPi;
constexpr auto kSqrt2 = constants<FloatingPoint>::kSqrt2;
constexpr auto kSqrt2Inv = constants<FloatingPoint>::kSqrt2Inv;
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_COMMON_H_
