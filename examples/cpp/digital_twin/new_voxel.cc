#include <iostream>

#include <wavemap/core/map/cell_types/voxel_data.h>

namespace {
struct DigitalTwinData {
  float r = 0.f;
  float g = 0.f;
  float b = 0.f;
  float traversability = 0.f;

  bool operator==(const DigitalTwinData& other) const {
    return r == other.r && g == other.g && b == other.b &&
           traversability == other.traversability;
  }
};

struct DigitalTwinDataPolicy {
  static DigitalTwinData add(const DigitalTwinData& lhs, const DigitalTwinData& rhs) {
    return {lhs.r + rhs.r, lhs.g + rhs.g, lhs.b + rhs.b,
            lhs.traversability + rhs.traversability};
  }

  static DigitalTwinData subtract(const DigitalTwinData& lhs,
                                  const DigitalTwinData& rhs) {
    return {lhs.r - rhs.r, lhs.g - rhs.g, lhs.b - rhs.b,
            lhs.traversability - rhs.traversability};
  }

  static DigitalTwinData scale(const DigitalTwinData& data,
                               wavemap::FloatingPoint factor) {
    return {factor * data.r, factor * data.g, factor * data.b,
            factor * data.traversability};
  }
};

void printVoxel(const char* label, const wavemap::VoxelData<DigitalTwinData, DigitalTwinDataPolicy>& voxel) {
  std::cout << label << " occupancy: " << voxel.occupancy << "\n";
  std::cout << label << " color: " << voxel.data.r << " " << voxel.data.g << " " << voxel.data.b << "\n";
  std::cout << label << " traversability: " << voxel.data.traversability << "\n";
}
}  // namespace

int main() {
  wavemap::VoxelData<> occupancy_only;
  occupancy_only = 1.0f;
  occupancy_only += 0.5f;

  std::cout << "Occupancy only: " << occupancy_only.occupancy << "\n";

  using DigitalTwinVoxel = wavemap::VoxelData<DigitalTwinData, DigitalTwinDataPolicy>;

  DigitalTwinVoxel digital_twin_voxel(0.8f, DigitalTwinData{1.f, 0.f, 0.f, 0.75f});
  digital_twin_voxel += 0.2f;
  printVoxel("Digital twin", digital_twin_voxel);

  wavemap::FloatingPoint occupancy_value = digital_twin_voxel;
  std::cout << "Implicit occupancy value: " << occupancy_value << "\n";

  const DigitalTwinVoxel voxel_a(0.4f, DigitalTwinData{1.f, 0.2f, 0.f, 0.3f});
  const DigitalTwinVoxel voxel_b(0.6f, DigitalTwinData{0.f, 0.3f, 1.f, 0.5f});

  printVoxel("Sum", voxel_a + voxel_b);
  printVoxel("Difference", voxel_b - voxel_a);
  printVoxel("Scaled", 0.5f * voxel_b);
  printVoxel("Divided", voxel_b / 2.f);

  return 0;
}
