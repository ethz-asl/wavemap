#include <iostream>
#include <optional>
#include <vector>

#include "../../common/example_layered_map_config.h"

namespace {
struct ExampleVoxelView {
  LayeredVoxel continuous;
  std::optional<int> semantic_label;
  std::optional<bool> changed;
};

ExampleVoxelView getExampleVoxel(const ExampleLayeredMap& map,
                                 const wavemap::Index3D& index) {
  return {map.continuousMap().getVoxelValue(index),
          map.discreteLayers().semantic.getValue(index),
          map.discreteLayers().changed.getValue(index)};
}

void printOptionalBool(const std::optional<bool>& value) {
  if (value) {
    std::cout << (*value ? "true" : "false");
  } else {
    std::cout << "none";
  }
}

void printLayeredMapVoxel(const std::string& name,
                           const ExampleVoxelView& voxel) {
  std::cout << name << ": occ=" << voxel.continuous.occupancy
            << " trav=" << voxel.continuous.data.traversability << " rgb=("
            << voxel.continuous.data.rgb.r << ", "
            << voxel.continuous.data.rgb.g << ", "
            << voxel.continuous.data.rgb.b << ") semantic=";
  if (voxel.semantic_label) {
    std::cout << *voxel.semantic_label;
  } else {
    std::cout << "none";
  }
  std::cout << " changed=";
  printOptionalBool(voxel.changed);
  std::cout << "\n";
}
}  // namespace

int main() {
  ExampleLayeredMapConfig config;
  config.continuous_map.min_cell_width = 0.1f;
  config.continuous_map.min_log_odds = -100.f;
  config.continuous_map.max_log_odds = 100.f;
  config.continuous_map.tree_height = 3;
  config.discrete_compression.block_height = 1;

  ExampleLayeredMap map(config);

  const wavemap::Index3D a(1, 2, 3);
  const wavemap::Index3D b(2, 2, 3);
  const wavemap::Index3D c(3, 2, 3);

  map.continuousMap().setVoxelValue(
      a, makeLayeredVoxel(10.f, rgb(1.f, 0.f, 0.f), 0.1f));
  map.continuousMap().setVoxelValue(
      b, makeLayeredVoxel(20.f, rgb(0.f, 1.f, 0.f), 0.5f));
  map.continuousMap().setVoxelValue(
      c, makeLayeredVoxel(30.f, rgb(0.f, 0.f, 1.f), 0.9f));

  map.discreteLayers().semantic.setValue(a, 1);
  map.discreteLayers().semantic.setValue(b, 5);
  map.discreteLayers().semantic.setValue(c, 9);

  map.discreteLayers().changed.setValue(a, false);
  map.discreteLayers().changed.setValue(b, true);
  map.discreteLayers().changed.setValue(c, false);

  std::cout << "LayeredMap experiment\n";
  std::cout << "Continuous fields are stored in the wavelet octree. "
               "Discrete semantic and binary layers are stored directly in "
               "compressed parent + exception layers, outside Haar arithmetic.\n";

  std::cout << "Before threshold/prune\n";
  printLayeredMapVoxel("a", getExampleVoxel(map, a));
  printLayeredMapVoxel("b", getExampleVoxel(map, b));
  printLayeredMapVoxel("c", getExampleVoxel(map, c));
  std::cout << "continuous nodes: " << map.continuousMap().size() << "\n";
  std::cout << "semantic compressed parents: " << map.discreteLayers().semantic.parentCount()
            << " exceptions: " << map.discreteLayers().semantic.exceptionCount()
            << " observed values: " << map.discreteLayers().semantic.observedValueCount()
            << "\n";
  std::cout << "changed compressed parents: " << map.discreteLayers().changed.parentCount()
            << " exceptions: " << map.discreteLayers().changed.exceptionCount()
            << " observed values: " << map.discreteLayers().changed.observedValueCount()
            << "\n";

  map.continuousMap().threshold();
  map.continuousMap().prune();

  std::cout << "After threshold/prune\n";
  printLayeredMapVoxel("a", getExampleVoxel(map, a));
  printLayeredMapVoxel("b", getExampleVoxel(map, b));
  printLayeredMapVoxel("c", getExampleVoxel(map, c));
  std::cout << "continuous nodes: " << map.continuousMap().size() << "\n";
  std::cout << "semantic compressed parents: " << map.discreteLayers().semantic.parentCount()
            << " exceptions: " << map.discreteLayers().semantic.exceptionCount()
            << " observed values: " << map.discreteLayers().semantic.observedValueCount()
            << "\n";
  std::cout << "changed compressed parents: " << map.discreteLayers().changed.parentCount()
            << " exceptions: " << map.discreteLayers().changed.exceptionCount()
            << " observed values: " << map.discreteLayers().changed.observedValueCount()
            << "\n";

  const bool semantic_labels_preserved = getExampleVoxel(map, a).semantic_label == 1 &&
                                         getExampleVoxel(map, b).semantic_label == 5 &&
                                         getExampleVoxel(map, c).semantic_label == 9;
  const bool changed_flags_preserved = getExampleVoxel(map, a).changed == false &&
                                       getExampleVoxel(map, b).changed == true &&
                                       getExampleVoxel(map, c).changed == false;
  if (!semantic_labels_preserved || !changed_flags_preserved) {
    std::cout << "Discrete layers were not preserved.\n";
    return 1;
  }
  return 0;
}
