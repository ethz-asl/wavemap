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

bool PinholeCameraProjectorConfig::isValid(bool verbose) const {
  bool is_valid = true;

  is_valid &= IS_PARAM_GT(width, 0, verbose);
  is_valid &= IS_PARAM_GT(height, 0, verbose);

  is_valid &= IS_PARAM_GT(fx, 0.f, verbose);
  is_valid &= IS_PARAM_GT(fy, 0.f, verbose);

  return is_valid;
}

PinholeCameraProjectorConfig PinholeCameraProjectorConfig::from(
    const param::Map& params) {
  PinholeCameraProjectorConfig config;

  for (const auto& [param_name, param_value] : params) {
    if (param_name == NAMEOF(width)) {
      config.width = param_value.get<IndexElement>();
    } else if (param_name == NAMEOF(height)) {
      config.height = param_value.get<IndexElement>();
    } else if (param_name == NAMEOF(fx)) {
      config.fx = param_value.get<FloatingPoint>();
    } else if (param_name == NAMEOF(fy)) {
      config.fy = param_value.get<FloatingPoint>();
    } else if (param_name == NAMEOF(cx)) {
      config.cx = param_value.get<FloatingPoint>();
    } else if (param_name == NAMEOF(cy)) {
      config.cy = param_value.get<FloatingPoint>();
    } else {
      LOG(WARNING) << "Ignoring unknown param with name " << param_name;
    }
  }

  return config;
}
}  // namespace wavemap
