#include <iostream>
#include <optional>

#include "../../common/example_layered_map_config.h"

namespace {
template <typename ValueT>
bool sameValue(const std::optional<ValueT>& value, const ValueT& expected) {
  return value && *value == expected;
}

bool expect(const std::string& name, bool condition) {
  std::cout << "  " << name << ": " << (condition ? "pass" : "FAIL")
            << "\n";
  return condition;
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
  const wavemap::Index3D a(0, 0, 0);
  const wavemap::Index3D b(1, 0, 0);
  const wavemap::Index3D c(1, 1, 1);

  map.continuousMap().setVoxelValue(
      a, makeLayeredVoxel(8.f, rgb(1.2f, -0.1f, 0.5f), 1.2f));
  map.continuousMap().setVoxelValue(
      b, makeLayeredVoxel(9.f, rgb(0.2f, 0.3f, 0.4f), 0.6f));
  map.continuousMap().setVoxelValue(
      c, makeLayeredVoxel(7.f, rgb(0.2f, 0.3f, 0.4f), 0.6f));

  map.discreteLayers().semantic.setValue(a, 1);
  map.discreteLayers().semantic.setValue(b, 1);
  map.discreteLayers().semantic.setValue(c, 2);
  map.discreteLayers().changed.setValue(a, false);
  map.discreteLayers().changed.setValue(b, false);
  map.discreteLayers().changed.setValue(c, true);

  map.continuousMap().threshold();
  map.continuousMap().prune();

  bool ok = true;
  std::cout << "LayeredMap continuous + discrete update behavior experiment\n";

  const LayeredVoxel voxel_a = map.continuousMap().getVoxelValue(a);
  ok &= expect("continuous threshold clamps rgb lower bound",
               0.f <= voxel_a.data.rgb.g);
  ok &= expect("continuous threshold clamps rgb upper bound",
               voxel_a.data.rgb.r <= 1.f);
  ok &= expect("continuous threshold clamps traversability",
               voxel_a.data.traversability <= 1.f);

  ok &= expect("semantic value a preserved",
               sameValue(map.discreteLayers().semantic.getValue(a), 1));
  ok &= expect("semantic value c preserved",
               sameValue(map.discreteLayers().semantic.getValue(c), 2));
  ok &= expect("binary value a preserved",
               sameValue(map.discreteLayers().changed.getValue(a), false));
  ok &= expect("binary value c preserved",
               sameValue(map.discreteLayers().changed.getValue(c), true));
  ok &= expect("unknown discrete voxel remains unknown",
               !map.discreteLayers().semantic.getValue(wavemap::Index3D(2, 2, 2)));
  ok &= expect("semantic layer compressed parent count is stable",
               map.discreteLayers().semantic.parentCount() == 1u);
  ok &= expect("semantic layer has one exception",
               map.discreteLayers().semantic.exceptionCount() == 1u);
  ok &= expect("changed layer has one exception",
               map.discreteLayers().changed.exceptionCount() == 1u);

  if (!ok) {
    std::cout << "LayeredMap integrated update behavior failed.\n";
    return 1;
  }
  return 0;
}
