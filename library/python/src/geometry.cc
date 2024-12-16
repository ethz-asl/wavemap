#include "pywavemap/geometry.h"

#include <nanobind/eigen/dense.h>
#include <wavemap/core/common.h>
#include <wavemap/core/utils/geometry/aabb.h>
#include <wavemap/core/utils/geometry/sphere.h>

using namespace nb::literals;  // NOLINT

namespace wavemap {
void add_geometry_bindings(nb::module_& m) {
  // Axis-Aligned Bounding Box
  nb::class_<AABB<Point3D>>(
      m, "AABB", "A class representing an Axis-Aligned Bounding Box.")
      .def(nb::init())
      .def(nb::init<Point3D, Point3D>(), "min"_a, "max"_a)
      .def_rw("min", &AABB<Point3D>::min)
      .def_rw("max", &AABB<Point3D>::max)
      .def("insert", &AABB<Point3D>::insert,
           "Expand the AABB to tightly fit the new point "
           "and its previous self.")
      .def("contains", &AABB<Point3D>::contains,
           "Test whether the AABB contains the given point.");

  // Axis-Aligned Bounding Box
  nb::class_<Sphere<Point3D>>(m, "Sphere", "A class representing a sphere.")
      .def(nb::init())
      .def(nb::init<Point3D, FloatingPoint>(), "center"_a, "radius"_a)
      .def_rw("center", &Sphere<Point3D>::center)
      .def_rw("radius", &Sphere<Point3D>::radius)
      .def("contains", &Sphere<Point3D>::contains,
           "Test whether the sphere contains the given point.");
}
}  // namespace wavemap
