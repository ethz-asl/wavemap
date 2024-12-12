#include "pywavemap/edit.h"

#include <memory>

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/unique_ptr.h>
#include <wavemap/core/map/hashed_chunked_wavelet_octree.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/core/utils/edit/crop.h>
#include <wavemap/core/utils/edit/multiply.h>
#include <wavemap/core/utils/edit/transform.h>

using namespace nb::literals;  // NOLINT

namespace wavemap {
void add_edit_module(nb::module_& m_edit) {
  // Map cropping methods
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

  // Map multiply methods
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

  // Map transformation methods
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
}
}  // namespace wavemap
