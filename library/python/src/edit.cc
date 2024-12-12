#include "pywavemap/edit.h"

#include <memory>

#include <nanobind/eigen/dense.h>
#include <wavemap/core/map/hashed_chunked_wavelet_octree.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/core/utils/edit/crop.h>

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
}
}  // namespace wavemap
