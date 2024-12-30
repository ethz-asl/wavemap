#include "wavemap/core/config/value_with_unit.h"

#include <map>
#include <string>
#include <utility>

namespace wavemap::param {
// clang-format off
static const std::map<std::string, std::pair<FloatingPoint, SiUnit::Id>> UnitToSi{
    {"kilometers",   {1e3f,        SiUnit::kMeters}},
    {"km",           {1e3f,        SiUnit::kMeters}},
    {"meters",       {1e0f,        SiUnit::kMeters}},
    {"m",            {1e0f,        SiUnit::kMeters}},
    {"decimeters",   {1e-1f,       SiUnit::kMeters}},
    {"dm",           {1e-1f,       SiUnit::kMeters}},
    {"centimeters",  {1e-2f,       SiUnit::kMeters}},
    {"cm",           {1e-2f,       SiUnit::kMeters}},
    {"millimeters",  {1e-3f,       SiUnit::kMeters}},
    {"mm",           {1e-3f,       SiUnit::kMeters}},
    {"radians",      {1.f,         SiUnit::kRadians}},
    {"rad",          {1.f,         SiUnit::kRadians}},
    {"degrees",      {kPi / 180.f, SiUnit::kRadians}},
    {"deg",          {kPi / 180.f, SiUnit::kRadians}},
    {"pixels",       {1.f,         SiUnit::kPixels}},
    {"px",           {1.f,         SiUnit::kPixels}},
    {"hours",        {3600.f,      SiUnit::kSeconds}},
    {"h",            {3600.f,      SiUnit::kSeconds}},
    {"minutes",      {60.f,        SiUnit::kSeconds}},
    {"seconds",      {1.f,         SiUnit::kSeconds}},
    {"sec",          {1.f,         SiUnit::kSeconds}},
    {"s",            {1.f,         SiUnit::kSeconds}},
    {"milliseconds", {1e-3f,       SiUnit::kSeconds}},
    {"ms",           {1e-3f,       SiUnit::kSeconds}},
    {"microseconds", {1e-6f,       SiUnit::kSeconds}},
    {"nanoseconds",  {1e-9f,       SiUnit::kSeconds}},
    {"ns",           {1e-9f,       SiUnit::kSeconds}},
};
// clang-format on

std::optional<ConversionToSi> getUnitToSi(const std::string& unit) {
  if (UnitToSi.count(unit)) {
    const auto [conversion_factor, si_unit] = UnitToSi.at(unit);
    return ConversionToSi{si_unit, conversion_factor};
  } else {
    return std::nullopt;
  }
}
}  // namespace wavemap::param
