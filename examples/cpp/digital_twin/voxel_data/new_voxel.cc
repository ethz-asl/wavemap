#include <iostream>

#include <wavemap/core/map/cell_types/voxel_data.h>

#include "../common/layered_voxel_config.h"

int main() {
  wavemap::VoxelData<> occupancy_only;
  occupancy_only = 1.0f;
  occupancy_only += 0.5f;

  std::cout << "Occupancy only: " << occupancy_only.occupancy << "\n";

  LayeredVoxel layered_voxel(0.8f, ContinuousLayers{rgb(1.f, 0.f, 0.f), 0.75f});
  layered_voxel += 0.2f;
  printVoxel("Layered voxel", layered_voxel);

  wavemap::FloatingPoint occupancy_value = layered_voxel;
  std::cout << "Implicit occupancy value: " << occupancy_value << "\n";

  const LayeredVoxel voxel_a(0.4f, ContinuousLayers{rgb(1.f, 0.2f, 0.f), 0.3f});
  const LayeredVoxel voxel_b(0.6f, ContinuousLayers{rgb(0.f, 0.3f, 1.f), 0.5f});

  printVoxel("Sum", voxel_a + voxel_b);
  printVoxel("Difference", voxel_b - voxel_a);
  printVoxel("Scaled", 0.5f * voxel_b);
  printVoxel("Divided", voxel_b / 2.f);

  return 0;
}
