#include "wavemap/config/unit_conversions.h"

namespace wavemap::param::convert {
// clang-format off
static const std::map<std::string, ValueWithUnit> UnitToSi{
    {"meters",      {1e0f,        SiUnit::kMeters}},
    {"m",           {1e0f,        SiUnit::kMeters}},
    {"decimeters",  {1e-1f,       SiUnit::kMeters}},
    {"dm",          {1e-1f,       SiUnit::kMeters}},
    {"centimeters", {1e-2f,       SiUnit::kMeters}},
    {"cm",          {1e-2f,       SiUnit::kMeters}},
    {"millimeters", {1e-3f,       SiUnit::kMeters}},
    {"mm",          {1e-3f,       SiUnit::kMeters}},
    {"radians",     {1.f,         SiUnit::kRadians}},
    {"rad",         {1.f,         SiUnit::kRadians}},
    {"degrees",     {kPi / 180.f, SiUnit::kRadians}},
    {"deg",         {kPi / 180.f, SiUnit::kRadians}},
    {"pixels",      {1.f,         SiUnit::kPixels}},
    {"px",          {1.f,         SiUnit::kPixels}},
    {"seconds",     {1.f,         SiUnit::kSeconds}},
    {"sec",         {1.f,         SiUnit::kSeconds}},
    {"s",           {1.f,         SiUnit::kSeconds}},
};
// clang-format on

ValueWithUnit toSiUnit(const Map& map) {
  if (map.size() != 1) {
    LOG(ERROR) << "Value with unit should have exactly one key and value.";
    return {kNaN, SiUnit::kInvalidTypeId};
  }

  const auto [unit, value] = *map.begin();
  if (!UnitToSi.count(unit)) {
    LOG(ERROR) << "Value has unknown unit " << unit << ".";
    return {kNaN, SiUnit::kInvalidTypeId};
  }

  if (!value.holds<FloatingPoint>()) {
    LOG(ERROR) << "Value is not of type float"
               << ".";
    return {kNaN, SiUnit::kInvalidTypeId};
  }

  return value.get<FloatingPoint>() * UnitToSi.at(unit);
}

FloatingPoint toUnit(const Map& map, SiUnit unit) {
  const auto value_with_unit = toSiUnit(map);
  if (value_with_unit.unit.toTypeId() != unit.toTypeId()) {
    LOG(FATAL) << "Expected unit of type " << unit.toStr() << ", but got "
               << value_with_unit.unit.toStr()
               << ". Returning NaN as no default is set.";
    return kNaN;
  }
  return value_with_unit.value;
}

FloatingPoint toUnit(const Value& param, SiUnit unit,
                     FloatingPoint default_value) {
  if (param.holds<Map>()) {
    const auto& sub_map = param.get<Map>();
    const auto value_with_unit = toSiUnit(sub_map);
    if (value_with_unit.unit == unit) {
      return value_with_unit.value;
    } else {
      LOG(ERROR) << "Expected unit of type " << unit.toStr() << ", but got "
                 << value_with_unit.value << " with unit "
                 << value_with_unit.unit.toStr() << ". Using default value "
                 << default_value << " instead.";
    }
  }
  return default_value;
}

FloatingPoint toUnit(const Map& map, const Name& key, SiUnit unit,
                     FloatingPoint default_value) {
  if (map::keyHoldsValue<Map>(map, key)) {
    const auto& sub_map = map::keyGetValue<Map>(map, key);
    const auto value_with_unit = toSiUnit(sub_map);
    if (value_with_unit.unit == unit) {
      return value_with_unit.value;
    } else {
      LOG(ERROR) << "Expected unit of type " << unit.toStr() << ", but got "
                 << value_with_unit.value << " with unit "
                 << value_with_unit.unit.toStr() << ". Using default value "
                 << default_value << " instead.";
    }
  }
  return default_value;
}
}  // namespace wavemap::param::convert
