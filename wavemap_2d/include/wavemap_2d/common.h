#ifndef WAVEMAP_2D_COMMON_H_
#define WAVEMAP_2D_COMMON_H_

#include <vector>

#include <Eigen/Eigen>
#include <kindr/minimal/quat-transformation.h>
#include <kindr/minimal/transform-2d.h>

namespace wavemap_2d {
using FloatingPoint = float;
constexpr FloatingPoint kEpsilon = 1e-6;

constexpr int MapDimension = 2;
using Transformation = kindr::minimal::Transformation2DTemplate<FloatingPoint>;
using Point = Transformation::Position;
using Vector = Transformation::Position;
using Rotation = Transformation::Rotation;

using Transformation3D =
    kindr::minimal::QuatTransformationTemplate<FloatingPoint>;
using Point3D = Transformation3D::Position;
using Vector3D = Transformation3D::Position;
using Rotation3D = Transformation3D::Rotation;

using IndexElement = int;
using Index = Eigen::Matrix<IndexElement, 2, 1>;
struct IndexHash {
  static constexpr size_t sl = 17191;

  std::size_t operator()(const Index& index) const {
    return static_cast<unsigned int>(index.x() + index.y() * sl);
  }
};

struct PointWithValue {
  Point position;
  FloatingPoint value;
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_COMMON_H_
