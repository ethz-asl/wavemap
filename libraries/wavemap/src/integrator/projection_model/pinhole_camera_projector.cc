#include "wavemap/integrator/projection_model/pinhole_camera_projector.h"

namespace wavemap {
AABB<Vector3D> PinholeCameraProjector::cartesianToSensorAABB(
    const AABB<Point3D>& W_aabb,
    const kindr::minimal::QuatTransformationTemplate<
        FloatingPoint>::RotationMatrix& R_C_W,
    const Point3D& t_W_C) const {
  const Point3D C_aabb_min = R_C_W * (W_aabb.min - t_W_C);
  const Transformation3D::RotationMatrix C_aabb_edges =
      R_C_W * (W_aabb.max - W_aabb.min).asDiagonal();

  std::array<Point3D, AABB<Point3D>::kNumCorners> C_aabb_corners;
  for (int corner_idx = 0; corner_idx < AABB<Point3D>::kNumCorners;
       ++corner_idx) {
    C_aabb_corners[corner_idx] = C_aabb_min;
    for (int dim_idx = 0; dim_idx < 3; ++dim_idx) {
      if ((corner_idx >> dim_idx) & 1) {
        C_aabb_corners[corner_idx] += C_aabb_edges.col(dim_idx);
      }
    }
  }

  std::array<Vector3D, AABB<Point3D>::kNumCorners> corner_sensor_coordinates;
  for (int corner_idx = 0; corner_idx < AABB<Point3D>::kNumCorners;
       ++corner_idx) {
    corner_sensor_coordinates[corner_idx] =
        cartesianToSensor(C_aabb_corners[corner_idx]);
  }

  AABB<Vector3D> sensor_coordinate_aabb;
  for (int dim_idx = 0; dim_idx < 3; ++dim_idx) {
    for (int corner_idx = 0; corner_idx < AABB<Point3D>::kNumCorners;
         ++corner_idx) {
      sensor_coordinate_aabb.min[dim_idx] =
          std::min(sensor_coordinate_aabb.min[dim_idx],
                   corner_sensor_coordinates[corner_idx][dim_idx]);
      sensor_coordinate_aabb.max[dim_idx] =
          std::max(sensor_coordinate_aabb.max[dim_idx],
                   corner_sensor_coordinates[corner_idx][dim_idx]);
    }
  }
  return sensor_coordinate_aabb;
}

DECLARE_CONFIG_MEMBERS(PinholeCameraProjectorConfig, (width), (height), (fx),
                       (fy), (cx), (cy));

bool PinholeCameraProjectorConfig::isValid(bool verbose) const {
  bool is_valid = true;

  is_valid &= IS_PARAM_GT(width, 0, verbose);
  is_valid &= IS_PARAM_GT(height, 0, verbose);

  is_valid &= IS_PARAM_GT(fx, 0.f, verbose);
  is_valid &= IS_PARAM_GT(fy, 0.f, verbose);

  return is_valid;
}
}  // namespace wavemap
