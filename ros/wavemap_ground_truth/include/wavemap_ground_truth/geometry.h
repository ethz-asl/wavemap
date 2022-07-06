#ifndef WAVEMAP_GROUND_TRUTH_GEOMETRY_H_
#define WAVEMAP_GROUND_TRUTH_GEOMETRY_H_

#include <limits>
#include <string>
#include <utility>

#include "wavemap_ground_truth/common.h"

namespace wavemap::ground_truth {
// NOTE: Many of the methods declared in this header are implemented based on
//       the examples in "Real-time collision detection" by Christer Ericson

struct AABB {
  Point3D min = Point3D::Constant(std::numeric_limits<FloatingPoint>::lowest());
  Point3D max = Point3D::Constant(std::numeric_limits<FloatingPoint>::max());

  std::string toString() const;
};

struct LineSegment {
  Point3D start_point = Point3D::Zero();
  Point3D end_point = Point3D::Zero();

  std::string toString() const;
};

struct Plane {
  Point3D normal;
  FloatingPoint offset_along_normal;

  bool intersectsLineSegment(const LineSegment& segment,
                             Point3D& intersection_point,
                             FloatingPoint& distance_to_intersection) const;
  bool intersectsLineSegment(const LineSegment& segment,
                             Point3D& intersection_point) const;
  bool intersectsLineSegment(const LineSegment& segment) const;

  std::string toString() const;
};

struct Triangle {
  Point3D vertex_a;
  Point3D vertex_b;
  Point3D vertex_c;

  AABB getAABB() const;
  FloatingPoint getDistanceToPoint(const Point3D& point) const;

  bool intersectsXAlignedRay(const Point& ray_origin_yz,
                             Point3D& barycentric_coordinates) const;
  bool intersectsXAlignedRay(const Point& ray_origin_yz) const;

  bool intersectsPlane(const Plane& plane,
                       LineSegment& intersecting_segment) const;
  bool intersectsPlane(const Plane& plane) const;

  std::string toString() const;

 private:
  static int getRelativeOrientation(const Point& vertex_one,
                                    const Point& vertex_two,
                                    FloatingPoint& twice_signed_area);
  static int getRelativeOrientation(const Point& vertex_one,
                                    const Point& vertex_two);
};
}  // namespace wavemap::ground_truth

#endif  // WAVEMAP_GROUND_TRUTH_GEOMETRY_H_
