#ifndef WAVEMAP_CORE_UTILS_SHAPE_INTERSECTION_TESTS_H_
#define WAVEMAP_CORE_UTILS_SHAPE_INTERSECTION_TESTS_H_

#include "wavemap/core/utils/shape/aabb.h"
#include "wavemap/core/utils/shape/sphere.h"

namespace wavemap::shape {
// Point inside Shape
template <typename ShapeT, int dim>
bool is_inside(const Point<dim>& inner, const ShapeT& outer) {
  return outer.contains(inner);
}

// AABB inside AABB
template <typename PointT>
bool is_inside(const AABB<PointT>& inner, const AABB<PointT>& outer) {
  return (outer.min.array() <= inner.min.array() &&
          inner.max.array() <= outer.max.array())
      .all();
}

// AABB inside Sphere
template <typename PointT>
bool is_inside(const AABB<PointT>& inner, const Sphere<PointT>& outer) {
  const PointT furthest_point_in_aabb = inner.furthestPointFrom(outer.center);
  return outer.contains(furthest_point_in_aabb);
}

// Sphere inside AABB
template <typename PointT>
bool is_inside(const Sphere<PointT>& inner, const AABB<PointT>& outer) {
  return (outer.min.array() <= inner.center.array() - inner.radius &&
          inner.center.array() + inner.radius <= outer.max.array())
      .all();
}

// AABB <-> AABB overlap
template <typename PointT>
bool overlaps(const AABB<PointT>& aabb_A, const AABB<PointT>& aabb_B) {
  const auto axis_separated = aabb_A.max.array() < aabb_B.min.array() ||
                              aabb_B.max.array() < aabb_A.min.array();
  return !axis_separated.any();
}

// AABB <-> Sphere overlap
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
