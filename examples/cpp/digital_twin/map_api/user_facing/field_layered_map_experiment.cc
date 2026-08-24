#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>

#include <wavemap/layered/continuous_layer_updater.h>

#include "../../common/field_layered_map_config.h"

namespace field_map = wavemap::examples::field_map;

namespace {
bool approximatelyEqual(float lhs, float rhs, float tolerance = 1e-5f) {
  return std::abs(lhs - rhs) <= tolerance;
}

bool expect(const std::string& label, bool condition) {
  std::cout << "  " << label << ": " << (condition ? "pass" : "FAIL")
            << "\n";
  return condition;
}
}  // namespace

int main() {
  field_map::Map map(field_map::makeDefaultConfig());
  const wavemap::Index3D index(2, -1, 3);
  map.continuousMap().setVoxelValue(
      index, field_map::Voxel(
                 0.7f, field_map::makeContinuousLayers(0.2f)));

  const field_map::Voxel before = map.continuousMap().getVoxelValue(index);
  const wavemap::layered::LayerObservation<float> observation(
      wavemap::Point3D::Zero(), 1.2f);
  const field_map::ReflectivityUpdatePolicy policy;
  wavemap::layered::updateContinuousLayersAtVoxel(
      map.continuousMap(), index, [&](field_map::ContinuousLayers& layers) {
        auto& reflectivity = layers.get<field_map::ReflectivityLayer>();
        reflectivity = policy(reflectivity, observation);
      });

  map.discreteLayers().get<field_map::ClassLayer>().setValue(
      index, static_cast<int>(field_map::ClassLabel::kObstacle));
  map.continuousMap().threshold();

  const field_map::Voxel after = map.continuousMap().getVoxelValue(index);
  const auto& configured_reflectivity_bounds =
      map.continuousMap()
          .getThresholdConfig()
          .data.get<field_map::ReflectivityLayer>();
  const auto class_value =
      map.discreteLayers().get<field_map::ClassLayer>().getValue(index);
  const auto schema = wavemap::layered::io::layeredMapSchema(map);

  bool ok = true;
  std::cout << "Field LayeredMap generated-schema experiment\n";
  ok &= expect("occupancy preserved",
               approximatelyEqual(before.occupancy, after.occupancy));
  ok &= expect("reflectivity thresholded to configured range",
               approximatelyEqual(
                   after.data.get<field_map::ReflectivityLayer>(),
                   configured_reflectivity_bounds.max));
  ok &= expect("class stored outside wavelet map",
               class_value &&
                   *class_value ==
                       static_cast<int>(field_map::ClassLabel::kObstacle));
  ok &= expect("schema has occupancy and reflectivity",
               schema.continuous_layers.size() == 2u &&
                   schema.continuous_layers[1].name == "reflectivity");
  ok &= expect("schema has one class layer",
               schema.discrete_layers.size() == 1u &&
                   schema.discrete_layers.front().name == "class");

  const std::filesystem::path file_path =
      "/tmp/field_layered_map_generated_schema.lwvmp";
  field_map::Map loaded(field_map::makeDefaultConfig());
  const auto* const continuous_map_before_load =
      loaded.continuousMapPtr().get();
  const bool saved_and_loaded =
      field_map::MapIo::save(file_path, map) &&
      field_map::MapIo::load(file_path, loaded);
  const auto loaded_voxel = loaded.continuousMap().getVoxelValue(index);
  const auto loaded_class =
      loaded.discreteLayers().get<field_map::ClassLayer>().getValue(index);
  ok &= expect(
      "persistence round trip",
      saved_and_loaded &&
          approximatelyEqual(
              loaded_voxel.data.get<field_map::ReflectivityLayer>(),
              configured_reflectivity_bounds.max) &&
          loaded_class &&
          *loaded_class ==
              static_cast<int>(field_map::ClassLabel::kObstacle));
  ok &= expect("load preserves continuous map allocation",
               loaded.continuousMapPtr().get() ==
                   continuous_map_before_load);
  const auto& loaded_reflectivity_bounds =
      loaded.continuousMap()
          .getThresholdConfig()
          .data.get<field_map::ReflectivityLayer>();
  ok &= expect("load preserves runtime layer thresholds",
               approximatelyEqual(loaded_reflectivity_bounds.min,
                                  configured_reflectivity_bounds.min) &&
                   approximatelyEqual(loaded_reflectivity_bounds.max,
                                      configured_reflectivity_bounds.max));

  return ok ? 0 : 1;
}
