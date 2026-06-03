#include <iostream>

#include <wavemap/core/map/cell_types/voxel_data.h>

namespace {
struct DigitalTwinData {
  float r = 0.f;
  float g = 0.f;
  float b = 0.f;
  float traversability = 0.f;

  bool operator==(const DigitalTwinData& other) const {
    return r == other.r && g == other.g && b == other.b && traversability == other.traversability;
  }
};
}  // namespace

int main() {
  wavemap::VoxelData<> occupancy_only;
  occupancy_only = 1.0f;
  occupancy_only += 0.5f;

  std::cout << "Occupancy only: " << occupancy_only.occupancy << "\n";

  wavemap::VoxelData<DigitalTwinData> digital_twin_voxel(
      0.8f, DigitalTwinData{1.f, 0.f, 0.f, 0.75f});
  digital_twin_voxel += 0.2f;

  std::cout << "Digital twin occupancy: " << digital_twin_voxel.occupancy
            << "\n";
  std::cout << "Color: " << digital_twin_voxel.data.r << " "
            << digital_twin_voxel.data.g << " "
            << digital_twin_voxel.data.b << "\n";
  std::cout << "Traversability: " << digital_twin_voxel.data.traversability << "\n";

  wavemap::FloatingPoint occupancy_value = digital_twin_voxel;
  std::cout << "Implicit occupancy value: " << occupancy_value << "\n";

  return 0;
}
