#include "wavemap/integrator/ray_tracing/ray_tracing_integrator.h"

namespace wavemap {
bool RayTracingIntegratorConfig::isValid(bool verbose) const {
  bool is_valid = true;

  is_valid &= IS_PARAM_GT(min_range, 0.f, verbose);
  is_valid &= IS_PARAM_GT(max_range, 0.f, verbose);
  is_valid &= IS_PARAM_LT(min_range, max_range, verbose);

  return is_valid;
}

RayTracingIntegratorConfig RayTracingIntegratorConfig::from(
    const param::Map& params) {
  RayTracingIntegratorConfig config;

  for (const auto& [param_name, param_value] : params) {
    if (param_name == NAMEOF(min_range)) {
      config.min_range =
          param::convert::toMeters(param_value, config.min_range);
    } else if (param_name == NAMEOF(max_range)) {
      config.max_range =
          param::convert::toMeters(param_value, config.max_range);
    } else {
      LOG(WARNING) << "Ignoring unknown param with name " << param_name;
    }
  }

  return config;
}

void RayTracingIntegrator::integratePointcloud(
    const PosedPointcloud<Point3D>& pointcloud) {
  if (!isPointcloudValid(pointcloud)) {
    return;
  }

  const FloatingPoint min_cell_width = occupancy_map_->getMinCellWidth();
  const Point3D& W_start_point = pointcloud.getOrigin();

  MeasurementModelType measurement_model(min_cell_width);
  measurement_model.setStartPoint(W_start_point);

  for (const auto& W_end_point : pointcloud.getPointsGlobal()) {
    measurement_model.setEndPoint(W_end_point);

    if (!isMeasurementValid(W_end_point - W_start_point)) {
      continue;
    }

    const FloatingPoint measured_distance =
        (W_start_point - W_end_point).norm();
    const Point3D W_end_point_truncated = getEndPointOrMaxRange(
        W_start_point, W_end_point, measured_distance, config_.max_range);
    const Ray ray(W_start_point, W_end_point_truncated, measured_distance);
    for (const auto& index : ray) {
      const FloatingPoint update = measurement_model.computeUpdate(index);
      occupancy_map_->addToCellValue(index, update);
    }
  }
}
}  // namespace wavemap
