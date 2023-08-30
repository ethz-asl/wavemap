#ifndef WAVEMAP_CONFIG_IMPL_VALUE_WITH_UNIT_INL_H_
#define WAVEMAP_CONFIG_IMPL_VALUE_WITH_UNIT_INL_H_

namespace wavemap {
template <SiUnit::Id unit, typename T>
ValueWithUnit<unit, T> ValueWithUnit<unit, T>::from(const param::Map& params) {
  if (params.size() != 1) {
    LOG(ERROR) << "Value with unit should have exactly one key and value.";
    return {};
  }

  const auto [read_unit, value] = *params.begin();

  const auto si_unit_and_conversion = param::getUnitToSi(read_unit);
  if (!si_unit_and_conversion.has_value()) {
    LOG(ERROR) << "Value has unknown unit " << read_unit << ".";
    return {};
  }

  const auto [si_unit, conversion_factor] = si_unit_and_conversion.value();
  if (si_unit != unit) {
    LOG(ERROR) << "Value has unit " << read_unit
               << " which cannot be converted to SI unit " << unit;
    return {};
  }

  if (value.holds<FloatingPoint>()) {
    return value.get<FloatingPoint>() * conversion_factor;
  }

  if (value.holds<int>()) {
    return static_cast<FloatingPoint>(value.get<int>()) * conversion_factor;
  }

  LOG(ERROR) << "Value must be of type float or int.";
  return {};
}
}  // namespace wavemap

#endif  // WAVEMAP_CONFIG_IMPL_VALUE_WITH_UNIT_INL_H_
