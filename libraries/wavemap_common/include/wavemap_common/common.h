#ifndef WAVEMAP_COMMON_COMMON_H_
#define WAVEMAP_COMMON_COMMON_H_

#include <vector>

#include <Eigen/Eigen>
#include <kindr/minimal/quat-transformation.h>
#include <kindr/minimal/transform-2d.h>

#include "wavemap_common/constants.h"

namespace wavemap {
using FloatingPoint = float;
template <typename T>
using MatrixT = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
using Matrix = MatrixT<FloatingPoint>;

using IndexElement = int;
template <int dim>
using Index = Eigen::Matrix<IndexElement, dim, 1>;
using Index2D = Index<2>;
using Index3D = Index<3>;
using LinearIndex = size_t;

template <int dim>
using Vector = Eigen::Matrix<FloatingPoint, dim, 1>;
using Vector2D = Vector<2>;
using Vector3D = Vector<3>;

template <int dim>
using Point = Vector<dim>;
using Point2D = Point<2>;
using Point3D = Point<3>;

using Transformation2D =
    kindr::minimal::Transformation2DTemplate<FloatingPoint>;
using Transformation3D =
    kindr::minimal::QuatTransformationTemplate<FloatingPoint>;
template <int dim>
using Transformation = typename std::conditional_t<
    dim == 2, Transformation2D,
    std::conditional_t<dim == 3, Transformation3D, std::false_type>>;

using Rotation2D = Transformation2D::Rotation;
using Rotation3D = Transformation3D::Rotation;
template <int dim>
using Rotation = typename std::conditional_t<
    dim == 2, Rotation2D,
    std::conditional_t<dim == 3, Rotation3D, std::false_type>>;

template <typename T>
inline constexpr int dim = T::RowsAtCompileTime;
template <>
inline constexpr int dim<Transformation2D> = 2;
template <>
inline constexpr int dim<Transformation3D> = 3;
template <>
inline constexpr int dim<Rotation2D> = 2;
template <>
inline constexpr int dim<Rotation3D> = 3;

constexpr auto kEpsilon = constants<FloatingPoint>::kEpsilon;
constexpr auto kPi = constants<FloatingPoint>::kPi;
constexpr auto kTwoPi = constants<FloatingPoint>::kTwoPi;
constexpr auto kHalfPi = constants<FloatingPoint>::kHalfPi;
constexpr auto kQuarterPi = constants<FloatingPoint>::kQuarterPi;
constexpr auto kSqrt2 = constants<FloatingPoint>::kSqrt2;
constexpr auto kSqrt2Inv = constants<FloatingPoint>::kSqrt2Inv;
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_COMMON_H_
