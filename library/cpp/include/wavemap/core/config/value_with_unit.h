#ifndef WAVEMAP_CORE_CONFIG_VALUE_WITH_UNIT_H_
#define WAVEMAP_CORE_CONFIG_VALUE_WITH_UNIT_H_

#include <string>

#include "wavemap/core/common.h"
#include "wavemap/core/config/param.h"
#include "wavemap/core/config/type_selector.h"

namespace wavemap {
struct SiUnit : TypeSelector<SiUnit> {
  using TypeSelector<SiUnit>::TypeSelector;

  enum Id : TypeId { kMeters, kRadians, kPixels, kSeconds };

  static constexpr std::array names = {"meters", "radians", "pixels",
                                       "seconds"};
};

template <SiUnit::Id unit, typename T>
struct ValueWithUnit {
  static constexpr SiUnit::Id kUnit = unit;
  T value{};

  // Constructors
  ValueWithUnit() = default;
  ValueWithUnit(T value) : value(value) {}  // NOLINT

  // Assignment operator
  ValueWithUnit& operator=(T rhs) {
    value = rhs;
    return *this;
  }

  // Allow implicit conversions to the underlying type
  operator T&() { return value; }
  operator const T&() const { return value; }

  // Method to load from configs
  static std::optional<ValueWithUnit> from(const param::Value& params);

  // Method to facilitate printing
  std::string toStr() const;
};

/**
 * Value expressed in meters.
 * @tparam T The value's underlying scalar type.
 */
template <typename T = FloatingPoint>
using Meters = ValueWithUnit<SiUnit::kMeters, T>;

/**
 * Value expressed in radians.
 * @tparam T The value's underlying scalar type.
 */
template <typename T = FloatingPoint>
using Radians = ValueWithUnit<SiUnit::kRadians, T>;

/**
 * Value expressed in pixels.
 * @tparam T The value's underlying scalar type.
 */
template <typename T = FloatingPoint>
using Pixels = ValueWithUnit<SiUnit::kPixels, T>;

/**
 * Value expressed in seconds.
 * @tparam T The value's underlying scalar type.
 */
template <typename T = FloatingPoint>
using Seconds = ValueWithUnit<SiUnit::kSeconds, T>;

namespace param {
struct ConversionToSi {
  SiUnit::Id si_unit;
  FloatingPoint conversion_factor;
};

std::optional<ConversionToSi> getUnitToSi(const std::string& unit);
}  // namespace param
}  // namespace wavemap

#include "wavemap/core/config/impl/value_with_unit_inl.h"

#endif  // WAVEMAP_CORE_CONFIG_VALUE_WITH_UNIT_H_
