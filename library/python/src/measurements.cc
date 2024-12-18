#include "pywavemap/measurements.h"

#include <nanobind/eigen/dense.h>
#include <wavemap/core/common.h>
#include <wavemap/core/data_structure/image.h>
#include <wavemap/core/data_structure/pointcloud.h>

using namespace nb::literals;  // NOLINT

namespace wavemap {
void add_measurement_bindings(nb::module_& m) {
  // Poses
  nb::class_<Rotation3D>(m, "Rotation",
                         "A class representing rotations in 3D space.")
      .def(nb::init<Rotation3D::RotationMatrix>(), "rotation_matrix"_a)
      .def(nb::init<FloatingPoint, FloatingPoint, FloatingPoint,
                    FloatingPoint>(),
           "w"_a, "x"_a, "y"_a, "z"_a)
      .def("inverse", &Rotation3D::inverse, "Compute the rotation's inverse.");
  nb::class_<Transformation3D>(m, "Pose",
                               "A class representing poses in 3D space.")
      .def(nb::init<Rotation3D, Vector3D>(), "rotation"_a, "translation"_a)
      .def(nb::init<Transformation3D::TransformationMatrix>(),
           "transformation_matrix"_a)
      .def("inverse", &Transformation3D::inverse,
           "Compute the pose's inverse.");

  // Pointclouds
  nb::class_<Pointcloud<>>(m, "Pointcloud", "A class to store pointclouds.")
      .def(nb::init<Pointcloud<>::Data>(), "point_matrix"_a)
      .def_prop_ro("size", &Pointcloud<>::size);
  nb::class_<PosedPointcloud<>>(
      m, "PosedPointcloud",
      "A class to store pointclouds with an associated pose.")
      .def(nb::init<Transformation3D, Pointcloud<>>(), "pose"_a,
           "pointcloud"_a);

  // Images
  nb::class_<Image<>>(m, "Image", "A class to store depth images.")
      .def(nb::init<Image<>::Data>(), "pixel_matrix"_a)
      .def_prop_ro("size", &Image<>::size)
      .def_prop_ro("dimensions", &Image<>::getDimensions);
  nb::class_<PosedImage<>>(
      m, "PosedImage", "A class to store depth images with an associated pose.")
      .def(nb::init<Transformation3D, Image<>>(), "pose"_a, "image"_a);
}
}  // namespace wavemap
