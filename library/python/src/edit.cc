#include "pywavemap/edit.h"

#include <memory>

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/unique_ptr.h>
#include <wavemap/core/map/hashed_chunked_wavelet_octree.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/core/utils/edit/crop.h>
#include <wavemap/core/utils/edit/multiply.h>
#include <wavemap/core/utils/edit/transform.h>
#include <wavemap/core/utils/geometry/aabb.h>
#include <wavemap/core/utils/geometry/sphere.h>

using namespace nb::literals;  // NOLINT

namespace wavemap {
void add_edit_module(nb::module_& m_edit) {
  // Multiply a map with a scalar
  // NOTE: Among others, this can be used to implement exponential forgetting,
  //       by multiplying the map with a scalar between 0 and 1.
  m_edit.def(
      "multiply",
      [](HashedWaveletOctree& map, FloatingPoint multiplier) {
        edit::multiply(map, multiplier, std::make_shared<ThreadPool>());
      },
      "map"_a, "multiplier"_a);
  m_edit.def(
      "multiply",
      [](HashedChunkedWaveletOctree& map, FloatingPoint multiplier) {
        edit::multiply(map, multiplier, std::make_shared<ThreadPool>());
      },
      "map"_a, "multiplier"_a);

  // Sum two maps together
  m_edit.def(
      "sum",
      [](HashedWaveletOctree& map_A, const HashedWaveletOctree& map_B) {
        edit::sum(map_A, map_B, std::make_shared<ThreadPool>());
      },
      "map_A"_a, "map_B"_a);
  m_edit.def(
      "sum",
      [](HashedChunkedWaveletOctree& map_A,
         const HashedChunkedWaveletOctree& map_B) {
        edit::sum(map_A, map_B, std::make_shared<ThreadPool>());
      },
      "map_A"_a, "map_B"_a);

  // Add a scalar value to all cells within an axis aligned bounding box
  m_edit.def(
      "sum",
      [](HashedWaveletOctree& map, const AABB<Point3D>& aabb,
         FloatingPoint update) {
        edit::sum(map, aabb, update, std::make_shared<ThreadPool>());
      },
      "map"_a, "aabb"_a, "update"_a);
  m_edit.def(
      "sum",
      [](HashedChunkedWaveletOctree& map, const AABB<Point3D>& aabb,
         FloatingPoint update) {
        edit::sum(map, aabb, update, std::make_shared<ThreadPool>());
      },
      "map"_a, "aabb"_a, "update"_a);

  // Add a scalar value to all cells within a sphere
  m_edit.def(
      "sum",
      [](HashedWaveletOctree& map, const Sphere<Point3D>& sphere,
         FloatingPoint update) {
        edit::sum(map, sphere, update, std::make_shared<ThreadPool>());
      },
      "map"_a, "sphere"_a, "update"_a);
  m_edit.def(
      "sum",
      [](HashedChunkedWaveletOctree& map, const Sphere<Point3D>& sphere,
         FloatingPoint update) {
        edit::sum(map, sphere, update, std::make_shared<ThreadPool>());
      },
      "map"_a, "sphere"_a, "update"_a);

  // Transform a map into a different coordinate frame
  m_edit.def(
      "transform",
      [](HashedWaveletOctree& B_map, const Transformation3D& T_AB) {
        return edit::transform(B_map, T_AB, std::make_shared<ThreadPool>());
      },
      "map"_a, "transformation"_a);
  m_edit.def(
      "transform",
      [](HashedChunkedWaveletOctree& B_map, const Transformation3D& T_AB) {
        return edit::transform(B_map, T_AB, std::make_shared<ThreadPool>());
      },
      "map"_a, "transformation"_a);

  // Crop a map
  m_edit.def(
      "crop_to_sphere",
      [](HashedWaveletOctree& map, const Point3D& t_W_center,
         FloatingPoint radius, IndexElement termination_height) {
        edit::crop_to_sphere(map, t_W_center, radius, termination_height,
                             std::make_shared<ThreadPool>());
      },
      "map"_a, "center_point"_a, "radius"_a, "termination_height"_a = 0);
  m_edit.def(
      "crop_to_sphere",
      [](HashedChunkedWaveletOctree& map, const Point3D& t_W_center,
         FloatingPoint radius, IndexElement termination_height) {
        edit::crop_to_sphere(map, t_W_center, radius, termination_height,
                             std::make_shared<ThreadPool>());
      },
      "map"_a, "center_point"_a, "radius"_a, "termination_height"_a = 0);
}
}  // namespace wavemap
