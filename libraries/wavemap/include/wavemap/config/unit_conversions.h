#ifndef WAVEMAP_CONFIG_UNIT_CONVERSIONS_H_
#define WAVEMAP_CONFIG_UNIT_CONVERSIONS_H_

#include "wavemap/common.h"
#include "wavemap/config/param.h"
#include "wavemap/config/type_selector.h"

namespace wavemap {
struct SiUnit : TypeSelector<SiUnit> {
  using TypeSelector<SiUnit>::TypeSelector;

  enum Id : TypeId { kMeters, kRadians, kPixels, kSeconds };

  static constexpr std::array names = {"meters", "radians", "pixels",
                                       "seconds"};
};

struct ValueWithUnit {
  FloatingPoint value = kNaN;
  SiUnit unit = SiUnit::kInvalidTypeId;
};
inline ValueWithUnit operator*(FloatingPoint lhs, const ValueWithUnit& rhs) {
  return {lhs * rhs.value, rhs.unit};
}

namespace param::convert {
ValueWithUnit toSiUnit(const Map& map);

FloatingPoint toUnit(const Map& map, SiUnit unit);

FloatingPoint toUnit(const Value& param, SiUnit unit,
                     FloatingPoint default_value);

FloatingPoint toUnit(const Map& map, const Name& key, SiUnit unit,
                     FloatingPoint default_value);

template <SiUnit::Id unit>
FloatingPoint toUnit(const Map& map) {
  return toUnit(map, unit);
}

template <SiUnit::Id unit>
FloatingPoint toUnit(const Value& param, FloatingPoint default_value) {
  return toUnit(param, unit, default_value);
}

template <SiUnit::Id unit>
FloatingPoint toUnit(const Map& map, const Name& key,
                     FloatingPoint default_value) {
  return toUnit(map, key, unit, default_value);
}
}  // namespace param::convert
}  // namespace wavemap

#endif  // WAVEMAP_CONFIG_UNIT_CONVERSIONS_H_
