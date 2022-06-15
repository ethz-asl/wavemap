#ifndef WAVEMAP_2D_COMMON_H_
#define WAVEMAP_2D_COMMON_H_

#include <vector>

#include <Eigen/Eigen>
#include <kindr/minimal/quat-transformation.h>
#include <kindr/minimal/transform-2d.h>

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

// NOTE: We define commonly used constants here instead of directly using the
//       mathematical constants from math.h in the code, mainly for consistency,
//       readability and to avoid unnecessary conversions at runtime. Such
//       conversions can for example be triggered when M_PI (double) is used
//       instead of M_PIf32 in an expression with floats, which reduces general
//       performance and also hinders things like auto-vectorization.
//       Defining our own constants here should also make things easier if we
//       ever want to evaluate different FloatingPoint precisions.
template <typename T = FloatingPoint>
struct constants {
  static constexpr auto kEpsilon = static_cast<T>(1e-6);
  static constexpr auto kPi = static_cast<T>(M_PI);
  static constexpr auto kTwoPi = static_cast<T>(2 * M_PI);
  static constexpr auto kHalfPi = static_cast<T>(M_PI_2);
  static constexpr auto kQuarterPi = static_cast<T>(M_PI_4);
  static constexpr auto kSqrt2 = static_cast<T>(M_SQRT2);
  static constexpr auto kSqrt2Inv = static_cast<T>(1 / M_SQRT2);
};
constexpr auto kEpsilon = constants<>::kEpsilon;
constexpr auto kPi = constants<>::kPi;
constexpr auto kTwoPi = constants<>::kTwoPi;
constexpr auto kHalfPi = constants<>::kHalfPi;
constexpr auto kQuarterPi = constants<>::kQuarterPi;
constexpr auto kSqrt2 = constants<>::kSqrt2;
constexpr auto kSqrt2Inv = constants<>::kSqrt2Inv;
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_COMMON_H_
