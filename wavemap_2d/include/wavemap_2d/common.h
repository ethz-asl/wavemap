#ifndef WAVEMAP_2D_COMMON_H_
#define WAVEMAP_2D_COMMON_H_

#include <vector>

#include <Eigen/Eigen>
#include <kindr/minimal/quat-transformation.h>
#include <kindr/minimal/transform-2d.h>

namespace wavemap_2d {
using FloatingPoint = float;
constexpr FloatingPoint kEpsilon = 1e-6f;
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
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_COMMON_H_
