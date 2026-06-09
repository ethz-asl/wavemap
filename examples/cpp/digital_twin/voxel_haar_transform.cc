#include <array>
#include <iostream>

#include <wavemap/core/map/cell_types/haar_coefficients.h>
#include <wavemap/core/map/cell_types/haar_transform.h>
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
  static DigitalTwinData add(const DigitalTwinData& lhs,
                             const DigitalTwinData& rhs) {
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

using DigitalTwinVoxel =
    wavemap::VoxelData<DigitalTwinData, DigitalTwinDataPolicy>;
using Coefficients = wavemap::HaarCoefficients<DigitalTwinVoxel, 3>;
using Transform = wavemap::HaarTransform<DigitalTwinVoxel, 3>;

void printVoxel(const char* label, const DigitalTwinVoxel& voxel) {
  std::cout << label << " occupancy: " << voxel.occupancy << "\n";
  std::cout << label << " color: " << voxel.data.r << " " << voxel.data.g
            << " " << voxel.data.b << "\n";
  std::cout << label << " traversability: "
            << voxel.data.traversability << "\n";
}
}  // namespace

int main() {
  Coefficients::CoefficientsArray child_voxels{
      DigitalTwinVoxel{0.1f, DigitalTwinData{1.f, 0.f, 0.f, 0.1f}},
      DigitalTwinVoxel{0.2f, DigitalTwinData{0.f, 1.f, 0.f, 0.2f}},
      DigitalTwinVoxel{0.3f, DigitalTwinData{0.f, 0.f, 1.f, 0.3f}},
      DigitalTwinVoxel{0.4f, DigitalTwinData{1.f, 1.f, 0.f, 0.4f}},
      DigitalTwinVoxel{0.5f, DigitalTwinData{1.f, 0.f, 1.f, 0.5f}},
      DigitalTwinVoxel{0.6f, DigitalTwinData{0.f, 1.f, 1.f, 0.6f}},
      DigitalTwinVoxel{0.7f, DigitalTwinData{0.5f, 0.5f, 0.5f, 0.7f}},
      DigitalTwinVoxel{0.8f, DigitalTwinData{1.f, 1.f, 1.f, 0.8f}},
  };

  const Coefficients::Parent parent_coefficients =
      Transform::forward(child_voxels);
  const Coefficients::CoefficientsArray reconstructed_children =
      Transform::backward(parent_coefficients);

  printVoxel("Parent scale", parent_coefficients.scale);
  printVoxel("First reconstructed child", reconstructed_children[0]);
  printVoxel("Last reconstructed child", reconstructed_children[7]);

  return 0;
}
