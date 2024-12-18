#ifndef WAVEMAP_CORE_UTILS_SHAPE_INTERSECTION_TESTS_H_
#define WAVEMAP_CORE_UTILS_SHAPE_INTERSECTION_TESTS_H_

#include "wavemap/core/utils/shape/aabb.h"
#include "wavemap/core/utils/shape/sphere.h"

namespace wavemap::shape {
template <typename ShapeT, int dim>
bool is_inside(const Point<dim>& point, const ShapeT& shape) {
  return shape.contains(point);
}

template <typename PointT>
bool is_inside(const AABB<PointT>& aabb, const Sphere<PointT>& sphere) {
  return sphere.contains(aabb.min) && sphere.contains(aabb.max);
}

template <typename PointT>
bool overlaps(const AABB<PointT>& aabb, const Sphere<PointT>& sphere) {
  const PointT closest_point_in_aabb = aabb.closestPointTo(sphere.center);
  return sphere.contains(closest_point_in_aabb);
}
template <typename PointT>
bool overlaps(const Sphere<PointT>& sphere, const AABB<PointT>& aabb) {
  return overlaps(aabb, sphere);
}
}  // namespace wavemap::shape

#endif  // WAVEMAP_CORE_UTILS_SHAPE_INTERSECTION_TESTS_H_
