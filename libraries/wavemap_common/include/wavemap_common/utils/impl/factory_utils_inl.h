#ifndef WAVEMAP_COMMON_UTILS_IMPL_FACTORY_UTILS_INL_H_
#define WAVEMAP_COMMON_UTILS_IMPL_FACTORY_UTILS_INL_H_

#include <string>

namespace wavemap {
template <typename DerivedNamedTypeSetT>
typename TypeSelector<DerivedNamedTypeSetT>::TypeId
TypeSelector<DerivedNamedTypeSetT>::strToTypeId(const std::string& name) {
  for (size_t type_idx = 0; type_idx < DerivedNamedTypeSetT::names.size();
       ++type_idx) {
    if (name == DerivedNamedTypeSetT::names[type_idx]) {
      return static_cast<TypeId>(type_idx);
    }
  }
  return kInvalidTypeId;
}

template <typename DerivedNamedTypeSetT>
DerivedNamedTypeSetT TypeSelector<DerivedNamedTypeSetT>::fromParamMap(
    const param::Map& params, std::string& error_msg) {
  if (!param::map::hasKey(params, "type")) {
    error_msg = "Type param is not set.";
    return kInvalidTypeId;
  }

  if (!param::map::keyHoldsValue<std::string>(params, "type")) {
    error_msg = "Type param is not a string.";
    return kInvalidTypeId;
  }

  const auto type_name = param::map::keyGetValue<std::string>(params, "type");
  DerivedNamedTypeSetT type_id(type_name);
  if (!type_id.isValid()) {
    error_msg = "Type param does not match a known type.";
  }

  return type_id;
}
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_UTILS_IMPL_FACTORY_UTILS_INL_H_
