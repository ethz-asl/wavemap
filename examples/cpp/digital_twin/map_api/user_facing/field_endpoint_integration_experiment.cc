#include <cmath>
#include <iostream>
#include <memory>

#include <wavemap/core/indexing/index_conversions.h>
#include <wavemap/core/integrator/ray_tracing/ray_tracing_integrator.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/layered/endpoint_attribute_integrator.h>

#include "../../common/field_layered_map_config.h"

namespace field_map = wavemap::examples::field_map;

int main() {
  const auto map_config = field_map::makeDefaultConfig();
  field_map::Map layered_map(map_config);
  auto baseline_map = std::make_shared<wavemap::HashedWaveletOctree>(
      map_config.continuous_map);
  wavemap::RayTracingIntegratorConfig config;
  config.min_range = 0.1f;
  config.max_range = 20.f;
  wavemap::RayTracingIntegrator occupancy_integrator(
      config, layered_map.continuousMapPtr());
  wavemap::RayTracingIntegrator baseline_integrator(config, baseline_map);

  using Measurement = wavemap::layered::PosedEndpointAttributePointcloud<
      field_map::ReflectivityLayer>;
  Measurement measurement{wavemap::Transformation3D()};
  measurement.resize(2u);
  measurement.point(0u) = wavemap::Point3D(1.1f, 0.1f, 0.1f);
  measurement.point(1u) = wavemap::Point3D(1.1f, 1.1f, 0.1f);
  measurement.attribute<field_map::ReflectivityLayer>(0u) = 0.25f;
  measurement.attribute<field_map::ReflectivityLayer>(1u) = 0.75f;

  wavemap::PosedPointcloud<> baseline_measurement(
      measurement.getPose(), measurement.points().data());
  baseline_integrator.integrate(baseline_measurement);
  wavemap::layered::EndpointAttributeIntegrator<
      field_map::Map, field_map::ReflectivityLayer>
      integrator(layered_map, occupancy_integrator);
  const auto result = integrator.integrate(measurement);

  const float cell_width_inv =
      1.f / layered_map.continuousMap().getMinCellWidth();
  bool occupancy_matches = true;
  bool reflectivity_matches = true;
  for (size_t i = 0u; i < measurement.size(); ++i) {
    const wavemap::Point3D endpoint = measurement.point(i);
    const auto index =
        wavemap::convert::pointToNearestIndex(endpoint, cell_width_inv);
    occupancy_matches &=
        std::abs(layered_map.continuousMap().getCellValue(index) -
                 baseline_map->getCellValue(index)) < 1e-6f;
    const auto voxel = layered_map.continuousMap().getVoxelValue(index);
    reflectivity_matches &=
        std::abs(voxel.data.template get<field_map::ReflectivityLayer>() -
                 measurement.attribute<field_map::ReflectivityLayer>(i)) <
        1e-6f;
  }
  const bool pass = result.integrated == 2u && occupancy_matches &&
                    reflectivity_matches;
  std::cout << "Field endpoint integration Phase 9 experiment\n"
            << "  original occupancy behavior preserved: "
            << (occupancy_matches ? "pass" : "FAIL") << '\n'
            << "  reflectivity updated at matching endpoints: "
            << (reflectivity_matches ? "pass" : "FAIL") << '\n';
  return pass ? 0 : 1;
}
