#include "wavemap_ground_truth/geometry.h"

#include <sstream>

#include <wavemap_common/utils/eigen_format.h>

namespace wavemap::ground_truth {
bool Plane::intersectsLineSegment(
    const LineSegment& segment, Point3D& intersection_point,
    FloatingPoint& distance_to_intersection) const {
  // Compute the distance_to_intersection value for the directed line ab
  // intersecting the plane
  Vector3D ab = segment.end_point - segment.start_point;
  distance_to_intersection =
      (offset_along_normal - normal.dot(segment.start_point)) / normal.dot(ab);
  // If distance_to_intersection in [0..1] compute and return intersection
  // point
  if (0.f <= distance_to_intersection && distance_to_intersection <= 1.f) {
    intersection_point = segment.start_point + distance_to_intersection * ab;
    return true;
  }
  // Else no intersection
  return false;
}

bool Plane::intersectsLineSegment(const LineSegment& segment,
                                  Point3D& intersection_point) const {
  FloatingPoint distance_to_intersection;
  return intersectsLineSegment(segment, intersection_point,
                               distance_to_intersection);
}

bool Plane::intersectsLineSegment(const LineSegment& segment) const {
  Point3D intersection_point;
  return intersectsLineSegment(segment, intersection_point);
}

AABB Triangle::getAABB() const {
  return {vertex_a.cwiseMin(vertex_b.cwiseMin(vertex_c)),
          vertex_a.cwiseMax(vertex_b.cwiseMax(vertex_c))};
}

FloatingPoint Triangle::getDistanceToPoint(const Point3D& point) const {
  // Check if the point is in the region outside A
  const Vector3D ab = vertex_b - vertex_a;
  const Vector3D ac = vertex_c - vertex_a;
  const Vector3D ap = point - vertex_a;
  const FloatingPoint d1 = ab.dot(ap);
  const FloatingPoint d2 = ac.dot(ap);
  if (d1 <= 0.0f && d2 <= 0.0f) {
    // The barycentric coordinates are (1,0,0) => the closest point is vertex_a
    return (vertex_a - point).norm();
  }

  // Check if P in vertex region outside B
  const Vector3D bp = point - vertex_b;
  const FloatingPoint d3 = ab.dot(bp);
  const FloatingPoint d4 = ac.dot(bp);
  if (d3 >= 0.0f && d4 <= d3) {
    // The barycentric coordinates are (0,1,0) => the closest point is vertex_b
    return (vertex_b - point).norm();
  }

  // Check if P in edge region of AB, if so return projection of P onto AB
  const FloatingPoint vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
    const FloatingPoint v = d1 / (d1 - d3);
    // The barycentric coordinates are (1-v,v,0)
    Point3D closest_pt = vertex_a + v * ab;
    return (closest_pt - point).norm();
  }

  // Check if P in vertex region outside C
  Vector3D cp = point - vertex_c;
  const FloatingPoint d5 = ab.dot(cp);
  const FloatingPoint d6 = ac.dot(cp);
  if (d6 >= 0.0f && d5 <= d6) {
    // The barycentric coordinates are (0,0,1) => the closest point is vertex_c
    return (vertex_c - point).norm();
  }

  // Check if P in edge region of AC, if so return projection of P onto AC
  const FloatingPoint vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
    const FloatingPoint w = d2 / (d2 - d6);
    // The barycentric coordinates are (1-w,0,w)
    const Point3D closest_pt = vertex_a + w * ac;
    return (closest_pt - point).norm();
  }

  // Check if P in edge region of BC, if so return projection of P onto BC
  const FloatingPoint va = d3 * d6 - d5 * d4;
  if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
    const FloatingPoint w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    // The barycentric coordinates are (0,1-w,w)
    const Point3D closest_pt = vertex_b + w * (vertex_c - vertex_b);
    return (closest_pt - point).norm();
  }

  // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
  const FloatingPoint denom = 1.0f / (va + vb + vc);
  const FloatingPoint v = vb * denom;
  const FloatingPoint w = vc * denom;
  // = u*a + v*b + w*c, u = va * denom = 1.0f - v - w
  const Point3D closest_pt = vertex_a + ab * v + ac * w;
  return (closest_pt - point).norm();
}

bool Triangle::intersectsXAlignedRay(const Point2D& ray_origin_yz,
                                     Point3D& barycentric_coordinates) const {
  // Express the vertices A, B and C relative to the point
  const Point2D vertex_a_relative = vertex_a.tail<2>() - ray_origin_yz;
  const Point2D vertex_b_relative = vertex_b.tail<2>() - ray_origin_yz;
  const Point2D vertex_c_relative = vertex_c.tail<2>() - ray_origin_yz;

  // Check the orientation of B relative to C
  // NOTE: As a byproduct of the orientation checks, we also compute the signed
  //       areas. After being normalized, these correspond to the barycentric
  //       coordinates (see the end of this method).
  const int sign_a = getRelativeOrientation(
      vertex_b_relative, vertex_c_relative, barycentric_coordinates[0]);
  // If the relative orientation is zero, vertices B and C must be equal.
  // This would mean that the triangle has no surface area and the ray therefore
  // cannot intersect it.
  if (sign_a == 0) return false;

  // Check the orientation of C relative to A
  const int sign_b = getRelativeOrientation(
      vertex_c_relative, vertex_a_relative, barycentric_coordinates[1]);
  // If the signs differ, the solution to the intersection equation does not lie
  // inside the triangle (i.e. the ray does not intersect the triangle)
  if (sign_b != sign_a) return false;

  // Check the orientation of A relative to B
  const int sign_c = getRelativeOrientation(
      vertex_a_relative, vertex_b_relative, barycentric_coordinates[2]);
  // If the signs differ, the solution to the intersection equation does not lie
  // inside the triangle (i.e. the ray does not intersect the triangle)
  if (sign_c != sign_a) return false;

  // If the point is within the triangle,
  // the barymetric coordinates should never be zero
  const FloatingPoint sum = barycentric_coordinates.sum();
  CHECK_NE(sum, 0);

  // Normalize the barycentric coordinates
  barycentric_coordinates /= sum;

  return true;
}

bool Triangle::intersectsXAlignedRay(const Point2D& ray_origin_yz) const {
  Point3D barycentric_coordinates;
  return intersectsXAlignedRay(ray_origin_yz, barycentric_coordinates);
}

bool Triangle::intersectsPlane(const Plane& plane,
                               LineSegment& intersecting_segment) const {
  int intersections_found = 0;
  for (const LineSegment& line_segment : {LineSegment{vertex_a, vertex_b},
                                          {vertex_b, vertex_c},
                                          {vertex_c, vertex_a}}) {
    Point3D intersection_point;
    if (plane.intersectsLineSegment(line_segment, intersection_point)) {
      ++intersections_found;
      if (intersections_found == 1) {
        intersecting_segment.start_point = intersection_point;
      } else {
        intersecting_segment.end_point = intersection_point;
      }
    }
  }

  CHECK_NE(intersections_found, 1)
      << "Found a triangle with only one edge that intersects with the plane. "
         "This is probably a numerical issue.";
  LOG_IF(WARNING, intersections_found == 3)
      << "Encountered a triangle that lies on the plane. Rasterizing this "
         "currently isn't supported.";
  return intersections_found;
}

bool Triangle::intersectsPlane(const Plane& plane) const {
  for (const LineSegment& line_segment : {LineSegment{vertex_a, vertex_b},
                                          {vertex_a, vertex_c},
                                          {vertex_b, vertex_c}}) {
    if (plane.intersectsLineSegment(line_segment)) {
      return true;
    }
  }
  return false;
}

int Triangle::getRelativeOrientation(const Point2D& vertex_one,
                                     const Point2D& vertex_two,
                                     FloatingPoint& twice_signed_area) {
  // Compute the signed area (scaled by factor 2, but we don't care)
  twice_signed_area =
      vertex_one[1] * vertex_two[0] - vertex_one[0] * vertex_two[1];

  // Return the relative orientation
  if (twice_signed_area > 0) {  // NOLINT
    return 1;
  } else if (twice_signed_area < 0) {  // NOLINT
    return -1;
  } else if (vertex_two[1] > vertex_one[1]) {
    return 1;
  } else if (vertex_two[1] < vertex_one[1]) {
    return -1;
  } else if (vertex_one[0] > vertex_two[0]) {
    return 1;
  } else if (vertex_one[0] < vertex_two[0]) {
    return -1;
  } else {
    return 0;
  }  // only true when both vertices are equal
}

int Triangle::getRelativeOrientation(const Point2D& vertex_one,
                                     const Point2D& vertex_two) {
  FloatingPoint twice_signed_area;
  return getRelativeOrientation(vertex_one, vertex_two, twice_signed_area);
}

std::string AABB::toString() const {
  std::ostringstream ss;
  ss << "AABB from min corner " << EigenFormat::oneLine(min)
     << " to max corner " << EigenFormat::oneLine(max);
  return ss.str();
}

std::string LineSegment::toString() const {
  std::ostringstream ss;
  ss << "line segment from point " << EigenFormat::oneLine(start_point)
     << " to " << EigenFormat::oneLine(end_point);
  return ss.str();
}

std::string Plane::toString() const {
  std::ostringstream ss;
  ss << "plane with normal " << EigenFormat::oneLine(normal) << " and offset "
     << offset_along_normal;
  return ss.str();
}

std::string Triangle::toString() const {
  std::ostringstream ss;
  ss << "triangle with vertices " << EigenFormat::oneLine(vertex_a) << ", "
     << EigenFormat::oneLine(vertex_b) << ", "
     << EigenFormat::oneLine(vertex_c);
  return ss.str();
}
}  // namespace wavemap::ground_truth
