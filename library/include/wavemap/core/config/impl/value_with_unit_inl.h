#ifndef WAVEMAP_CORE_CONFIG_IMPL_VALUE_WITH_UNIT_INL_H_
#define WAVEMAP_CORE_CONFIG_IMPL_VALUE_WITH_UNIT_INL_H_

#include <string>

namespace wavemap {
template <SiUnit::Id unit, typename T>
std::string ValueWithUnit<unit, T>::toStr() const {
  return std::to_string(value) + " [" + SiUnit::toStr(unit) + "]";
}

template <SiUnit::Id unit, typename T>
std::optional<ValueWithUnit<unit, T>> ValueWithUnit<unit, T>::from(
    const param::Value& params) {
  const auto param_map = params.as<param::Map>();
  if (!param_map) {
    LOG(WARNING) << "Tried to load a value with annotated units from a param "
                    "that is not of type Map (dictionary). Cannot perform "
                    "conversion to SI unit \""
                 << SiUnit::toStr(unit) << "\". Will be ignored.";
    return std::nullopt;
  }

  if (param_map->size() != 1) {
    LOG(WARNING) << "Value with unit should have exactly one key and value. "
                    "Cannot perform conversion to SI unit \""
                 << SiUnit::toStr(unit) << "\". Will be ignored.";
    return std::nullopt;
  }

  const auto [param_unit, param_value] = *param_map->begin();

  const auto si_unit_and_conversion = param::getUnitToSi(param_unit);
  if (!si_unit_and_conversion) {
    LOG(WARNING) << "Value has unknown unit \"" << param_unit
                 << "\". Cannot perform conversion to SI unit \""
                 << SiUnit::toStr(unit) << "\". Will be ignored.";
    return std::nullopt;
  }

  const auto [si_unit, conversion_factor] = si_unit_and_conversion.value();
  if (si_unit != unit) {
    LOG(WARNING) << "Value has unit \"" << param_unit
                 << "\" which cannot be converted to SI unit \""
                 << SiUnit::toStr(unit) << "\". Will be ignored.";
    return std::nullopt;
  }

  if (const auto param_float = param_value.as<FloatingPoint>(); param_float) {
    return param_float.value() * conversion_factor;
  }

  if (const auto param_int = param_value.as<int>(); param_int) {
    return static_cast<FloatingPoint>(param_int.value()) * conversion_factor;
  }

  LOG(WARNING) << "Value must be of type float or int. Cannot perform "
                  "conversion to SI unit \""
               << SiUnit::toStr(unit) << "\". Will be ignored.";
  return std::nullopt;
}
}  // namespace wavemap

#endif  // WAVEMAP_CORE_CONFIG_IMPL_VALUE_WITH_UNIT_INL_H_
