#include "pywavemap/raycast.h"

#include <nanobind/eigen/dense.h>  // to use eigen2numpy seamlessly
#include <wavemap/core/common.h>
#include <wavemap/core/data_structure/image.h>
#include <wavemap/core/integrator/projection_model/pinhole_camera_projector.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/core/utils/iterate/grid_iterator.h>
#include <wavemap/core/utils/query/query_accelerator.h>
#include <wavemap/core/utils/thread_pool.h>

#include "wavemap/core/utils/iterate/ray_iterator.h"

using namespace nb::literals;  // NOLINT

namespace wavemap {
FloatingPoint raycast(const HashedWaveletOctree& map,
                      const Point3D& start_point, const Point3D& end_point,
                      FloatingPoint log_odds_threshold) {
  const FloatingPoint mcw = map.getMinCellWidth();
  const Ray ray(start_point, end_point, mcw);
  for (const Index3D& ray_voxel_index : ray) {
    if (log_odds_threshold < map.getCellValue(ray_voxel_index)) {
      const Point3D voxel_center =
          convert::indexToCenterPoint(ray_voxel_index, mcw);
      return (voxel_center - start_point).norm();
    }
  }
  return (end_point - start_point).norm();
}

FloatingPoint raycast_fast(
    QueryAccelerator<HashedWaveletOctree>& query_accelerator,
    const Point3D& start_point, const Point3D& end_point,
    FloatingPoint log_odds_threshold) {
  const FloatingPoint min_cell_width = query_accelerator.getMinCellWidth();
  const Ray ray(start_point, end_point, min_cell_width);
  for (const Index3D& ray_voxel_index : ray) {
    if (log_odds_threshold < query_accelerator.getCellValue(ray_voxel_index)) {
      const Point3D voxel_center =
          convert::indexToCenterPoint(ray_voxel_index, min_cell_width);
      return (voxel_center - start_point).norm();
    }
  }
  return (end_point - start_point).norm();
}

void add_raycast_bindings(nb::module_& m) {
  nb::class_<PinholeCameraProjectorConfig>(
      m, "PinholeCameraProjectorConfig", "Describes pinhole camera intrinsics")
      .def(nb::init<FloatingPoint, FloatingPoint, FloatingPoint, FloatingPoint,
                    IndexElement, IndexElement>(),
           "fx"_a, "fy"_a, "cx"_a, "cy"_a, "height"_a, "width"_a)
      .def_rw("width", &PinholeCameraProjectorConfig::width)
      .def_rw("height", &PinholeCameraProjectorConfig::height)
      .def_rw("fx", &PinholeCameraProjectorConfig::fx)
      .def_rw("fy", &PinholeCameraProjectorConfig::fy)
      .def_rw("cx", &PinholeCameraProjectorConfig::cx)
      .def_rw("cy", &PinholeCameraProjectorConfig::cy)
      .def("__repr__", [](const PinholeCameraProjectorConfig& self) {
        return nb::str(
                   "PinholeCameraProjectorConfig(width={}, height={}, fx={}, "
                   "fy={}, cx={}, cy={})")
            .format(self.width, self.height, self.fx, self.fy, self.cx,
                    self.cy);
      });

  m.def("raycast", &raycast,
        "Raycast and get first point with occopancy higher than threshold");

  m.def("raycast_fast",
        &raycast_fast,  // TODO: unusable without QueryAccelerator binding
        "Raycast and get first point with occopancy higher than threshold "
        "using QueryAccelerator for efficiency");

  m.def(
      "get_depth",
      [](const HashedWaveletOctree& map, const Transformation3D& pose,
         const PinholeCameraProjectorConfig& cam_cfg,
         FloatingPoint log_odds_threshold, FloatingPoint max_range) {
        const PinholeCameraProjector projection_model(cam_cfg);
        Image depth_image(projection_model.getDimensions());
        const Point3D& W_start_point = pose.getPosition();

        ThreadPool thread_pool;
        constexpr int kPatchWidth = 64;
        const Index2D num_patches =
            int_math::div_round_up(depth_image.getDimensions(), kPatchWidth);
        for (const Index2D& patch_index :
             Grid<2>(Index2D::Zero(), num_patches - Index2D::Ones())) {
          thread_pool.add_task([&map, &projection_model, &pose, &W_start_point,
                                &depth_image, patch_index, kPatchWidth,
                                max_range, log_odds_threshold]() {
            QueryAccelerator query_accelerator(map);
            const Index2D patch_min = patch_index * kPatchWidth;
            const Index2D patch_max =
                (patch_min.array() + kPatchWidth)
                    .min(depth_image.getDimensions().array()) -
                1;
            for (const Index2D& index : Grid<2>(patch_min, patch_max)) {
              FloatingPoint& depth_pixel = depth_image.at(index);
              const Vector2D image_xy = projection_model.indexToImage(index);
              const Point3D C_end_point =
                  projection_model.sensorToCartesian({image_xy, max_range});
              const Point3D W_end_point = pose * C_end_point;
              depth_pixel = raycast_fast(query_accelerator, W_start_point,
                                         W_end_point, log_odds_threshold);
            }
          });
        }
        thread_pool.wait_all();

        return depth_image.getData().transpose().eval();
      },
      "Extract depth from octree map at using given camera pose and "
      "intrinsics");
}
}  // namespace wavemap
