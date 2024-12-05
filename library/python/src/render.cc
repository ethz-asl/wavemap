#include "pywavemap/render.h"

#include <memory>

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/shared_ptr.h>
#include <wavemap/core/common.h>
#include <wavemap/core/utils/render/raycasting_renderer.h>

using namespace nb::literals;  // NOLINT

namespace wavemap {
void add_render_bindings(nb::module_& m) {
  nb::class_<RaycastingRenderer>(m, "RaycastingRenderer",
                                 "Render depth images using ray casting.")
      .def(nb::init<std::shared_ptr<const MapBase>,
                    std::shared_ptr<const ProjectorBase>, FloatingPoint,
                    FloatingPoint, FloatingPoint, FloatingPoint>(),
           "occupancy_map"_a, "projection_model"_a,
           "log_odds_occupancy_threshold"_a = 1e-3f, "min_range"_a = 0.f,
           "max_range"_a = 10.f, "default_depth_value"_a = -1.f)
      .def("render", &RaycastingRenderer::render, "pose"_a);
}
}  // namespace wavemap
