#include "pywavemap/measurements.h"

#include <nanobind/eigen/dense.h>
#include <wavemap/core/common.h>
#include <wavemap/core/data_structure/image.h>
#include <wavemap/core/data_structure/pointcloud.h>

using namespace nb::literals;  // NOLINT

namespace wavemap {
void add_measurement_bindings(nb::module_& m) {
  // Poses
  nb::class_<Rotation3D>(m, "Rotation")
      .def(nb::init<Rotation3D::RotationMatrix>(), "rotation_matrix"_a);
  nb::class_<Transformation3D>(m, "Pose")
      .def(nb::init<Rotation3D, Vector3D>(), "rotation"_a, "translation"_a)
      .def(nb::init<Transformation3D::TransformationMatrix>(),
           "transformation_matrix");

  // Pointclouds
  nb::class_<Pointcloud<>>(m, "Pointcloud")
      .def(nb::init<Pointcloud<>::Data>(), "point_matrix"_a);
  nb::class_<PosedPointcloud<>>(m, "PosedPointcloud")
      .def(nb::init<Transformation3D, Pointcloud<>>(), "pose"_a,
           "pointcloud"_a);

  // Images
  nb::class_<Image<>>(m, "Image")
      .def(nb::init<Image<>::Data>(), "pixel_matrix"_a);
  nb::class_<PosedImage<>>(m, "PosedImage")
      .def(nb::init<Transformation3D, Image<>>(), "pose"_a, "image"_a);
}
}  // namespace wavemap
