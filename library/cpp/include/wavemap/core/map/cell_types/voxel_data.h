#ifndef WAVEMAP_CORE_DATA_STRUCTURE_VOXEL_DATA_H_
#define WAVEMAP_CORE_DATA_STRUCTURE_VOXEL_DATA_H_

#include "wavemap/core/common.h"

namespace wavemap {
/**
 * @brief A template structure for storing voxel data with mandatory occupancy
 *        and arbitrary additional data.
 *
 * This template allows users to store custom data alongside occupancy
 * information in each voxel without compromising efficiency.
 *
 * @tparam AdditionalDataT The type of additional data to store in each voxel.
 *                          Use void (default) for occupancy-only storage.
 *
 * Example usage:
 * @code
 *   // Voxel with occupancy only (same as FloatingPoint)
 *   using BasicVoxel = VoxelData<void>;
 *
 *   // Voxel with occupancy + color
 *   struct ColorData {
 *     uint8_t r, g, b;
 *   };
 *   using ColorVoxel = VoxelData<ColorData>;
 *
 *   // Voxel with occupancy + semantic label
 *   using SemanticVoxel = VoxelData<uint8_t>;
 * @endcode
 */
template <typename AdditionalDataT = void>
struct VoxelData {
  FloatingPoint occupancy = 0.f;
  AdditionalDataT data{};

  // Default constructor
  VoxelData() = default;

  // Constructor from occupancy only
  explicit VoxelData(FloatingPoint occupancy_value)
      : occupancy(occupancy_value) {}

  // Constructor from occupancy and additional data
  VoxelData(FloatingPoint occupancy_value, const AdditionalDataT& data_value)
      : occupancy(occupancy_value), data(data_value) {}

  // Implicit conversion to FloatingPoint for backward compatibility
  operator FloatingPoint() const { return occupancy; }

  // Assignment from FloatingPoint
  VoxelData& operator=(FloatingPoint value) {
    occupancy = value;
    return *this;
  }

  // Equality operators
  bool operator==(const VoxelData& other) const {
    return occupancy == other.occupancy && data == other.data;
  }

  bool operator!=(const VoxelData& other) const { return !(*this == other); }

  // Allow use with += operator for occupancy updates
  VoxelData& operator+=(FloatingPoint value) {
    occupancy += value;
    return *this;
  }

  // Addition operator
  VoxelData operator+(FloatingPoint value) const {
    VoxelData result = *this;
    result.occupancy += value;
    return result;
  }
};

/**
 * @brief Specialization for void additional data (occupancy-only).
 *
 * This specialization provides the same interface as the general template
 * but stores only the occupancy value, making it fully compatible with
 * FloatingPoint while still allowing future extension.
 */
template <>
struct VoxelData<void> {
  FloatingPoint occupancy = 0.f;

  // Default constructor
  VoxelData() = default;

  // Constructor from occupancy
  explicit VoxelData(FloatingPoint occupancy_value)
      : occupancy(occupancy_value) {}

  // Implicit conversion to FloatingPoint
  operator FloatingPoint() const { return occupancy; }

  // Assignment from FloatingPoint
  VoxelData& operator=(FloatingPoint value) {
    occupancy = value;
    return *this;
  }

  // Equality operators
  bool operator==(const VoxelData& other) const {
    return occupancy == other.occupancy;
  }

  bool operator!=(const VoxelData& other) const { return !(*this == other); }

  // Allow use with += operator
  VoxelData& operator+=(FloatingPoint value) {
    occupancy += value;
    return *this;
  }

  // Addition operator
  VoxelData operator+(FloatingPoint value) const {
    VoxelData result = *this;
    result.occupancy += value;
    return result;
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_CORE_DATA_STRUCTURE_VOXEL_DATA_H_