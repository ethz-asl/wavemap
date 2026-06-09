#ifndef WAVEMAP_CORE_MAP_CELL_TYPES_VOXEL_DATA_H_
#define WAVEMAP_CORE_MAP_CELL_TYPES_VOXEL_DATA_H_

#include "wavemap/core/common.h"

namespace wavemap {
template <typename AdditionalDataT>
struct DefaultVoxelDataPolicy {
  static AdditionalDataT add(const AdditionalDataT& lhs,
                             const AdditionalDataT& rhs) {
    return lhs + rhs;
  }

  static AdditionalDataT subtract(const AdditionalDataT& lhs,
                                  const AdditionalDataT& rhs) {
    return lhs - rhs;
  }

  static AdditionalDataT scale(const AdditionalDataT& data,
                               FloatingPoint factor) {
    return data * factor;
  }
};

template <>
struct DefaultVoxelDataPolicy<void> {};

/**
 * @brief Voxel data with mandatory occupancy and optional user data.
 *
 * AdditionalDataPolicyT defines the arithmetic of the additional data when this
 * type is used in numerical algorithms such as wavelet transforms.
 */
template <typename AdditionalDataT = void,
          typename AdditionalDataPolicyT =
              DefaultVoxelDataPolicy<AdditionalDataT>>
struct VoxelData {
  using AdditionalData = AdditionalDataT;
  using AdditionalDataPolicy = AdditionalDataPolicyT;

  FloatingPoint occupancy = 0.f;
  AdditionalDataT data{};

  VoxelData() = default;
  explicit VoxelData(FloatingPoint occupancy_value)
      : occupancy(occupancy_value) {}
  VoxelData(FloatingPoint occupancy_value, const AdditionalDataT& data_value)
      : occupancy(occupancy_value), data(data_value) {}

  operator FloatingPoint() const { return occupancy; }  // NOLINT

  VoxelData& operator=(FloatingPoint value) {
    occupancy = value;
    return *this;
  }

  bool operator==(const VoxelData& other) const {
    return occupancy == other.occupancy && data == other.data;
  }
  bool operator!=(const VoxelData& other) const { return !(*this == other); }

  VoxelData& operator+=(FloatingPoint value) {
    occupancy += value;
    return *this;
  }

  VoxelData& operator+=(const VoxelData& other) {
    occupancy += other.occupancy;
    data = AdditionalDataPolicyT::add(data, other.data);
    return *this;
  }

  VoxelData& operator-=(const VoxelData& other) {
    occupancy -= other.occupancy;
    data = AdditionalDataPolicyT::subtract(data, other.data);
    return *this;
  }

  VoxelData& operator*=(FloatingPoint factor) {
    occupancy *= factor;
    data = AdditionalDataPolicyT::scale(data, factor);
    return *this;
  }

  VoxelData& operator/=(FloatingPoint factor) {
    occupancy /= factor;
    data = AdditionalDataPolicyT::scale(data, 1.f / factor);
    return *this;
  }

  VoxelData& operator*=(const VoxelData& scalar) {
    return *this *= scalar.occupancy;
  }

  VoxelData& operator/=(const VoxelData& scalar) {
    return *this /= scalar.occupancy;
  }

  VoxelData operator+(FloatingPoint value) const {
    VoxelData result = *this;
    result.occupancy += value;
    return result;
  }

  friend VoxelData operator+(VoxelData lhs, const VoxelData& rhs) {
    lhs += rhs;
    return lhs;
  }

  friend VoxelData operator-(VoxelData lhs, const VoxelData& rhs) {
    lhs -= rhs;
    return lhs;
  }

  friend VoxelData operator+(VoxelData value) { return value; }

  friend VoxelData operator-(VoxelData value) {
    value.occupancy = -value.occupancy;
    value.data = AdditionalDataPolicyT::scale(value.data, -1.f);
    return value;
  }

  friend VoxelData operator*(FloatingPoint factor, VoxelData value) {
    value *= factor;
    return value;
  }

  friend VoxelData operator*(VoxelData value, FloatingPoint factor) {
    value *= factor;
    return value;
  }

  // Supports generic numeric code that represents scalars as ValueT objects,
  // e.g. static_cast<VoxelData>(0.5f) * voxel in Haar transforms.
  friend VoxelData operator*(const VoxelData& scalar, VoxelData value) {
    value *= scalar.occupancy;
    return value;
  }

  friend VoxelData operator/(VoxelData value, FloatingPoint factor) {
    value /= factor;
    return value;
  }

  friend VoxelData operator/(VoxelData value, const VoxelData& scalar) {
    value /= scalar.occupancy;
    return value;
  }
};

/**
 * @brief Occupancy-only specialization.
 */
template <typename AdditionalDataPolicyT>
struct VoxelData<void, AdditionalDataPolicyT> {
  FloatingPoint occupancy = 0.f;

  VoxelData() = default;
  explicit VoxelData(FloatingPoint occupancy_value)
      : occupancy(occupancy_value) {}

  operator FloatingPoint() const { return occupancy; }  // NOLINT

  VoxelData& operator=(FloatingPoint value) {
    occupancy = value;
    return *this;
  }

  bool operator==(const VoxelData& other) const {
    return occupancy == other.occupancy;
  }
  bool operator!=(const VoxelData& other) const { return !(*this == other); }

  VoxelData& operator+=(FloatingPoint value) {
    occupancy += value;
    return *this;
  }

  VoxelData& operator+=(const VoxelData& other) {
    occupancy += other.occupancy;
    return *this;
  }

  VoxelData& operator-=(const VoxelData& other) {
    occupancy -= other.occupancy;
    return *this;
  }

  VoxelData& operator*=(FloatingPoint factor) {
    occupancy *= factor;
    return *this;
  }

  VoxelData& operator/=(FloatingPoint factor) {
    occupancy /= factor;
    return *this;
  }

  VoxelData& operator*=(const VoxelData& scalar) {
    occupancy *= scalar.occupancy;
    return *this;
  }

  VoxelData& operator/=(const VoxelData& scalar) {
    occupancy /= scalar.occupancy;
    return *this;
  }

  VoxelData operator+(FloatingPoint value) const {
    VoxelData result = *this;
    result.occupancy += value;
    return result;
  }

  friend VoxelData operator+(VoxelData lhs, const VoxelData& rhs) {
    lhs += rhs;
    return lhs;
  }

  friend VoxelData operator-(VoxelData lhs, const VoxelData& rhs) {
    lhs -= rhs;
    return lhs;
  }

  friend VoxelData operator+(VoxelData value) { return value; }

  friend VoxelData operator-(VoxelData value) {
    value.occupancy = -value.occupancy;
    return value;
  }

  friend VoxelData operator*(FloatingPoint factor, VoxelData value) {
    value *= factor;
    return value;
  }

  friend VoxelData operator*(VoxelData value, FloatingPoint factor) {
    value *= factor;
    return value;
  }

  friend VoxelData operator*(const VoxelData& lhs, VoxelData rhs) {
    rhs *= lhs.occupancy;
    return rhs;
  }

  friend VoxelData operator/(VoxelData value, FloatingPoint factor) {
    value /= factor;
    return value;
  }

  friend VoxelData operator/(VoxelData value, const VoxelData& scalar) {
    value /= scalar.occupancy;
    return value;
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_CORE_MAP_CELL_TYPES_VOXEL_DATA_H_
