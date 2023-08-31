#ifndef WAVEMAP_CONFIG_IMPL_TYPE_SELECTOR_INL_H_
#define WAVEMAP_CONFIG_IMPL_TYPE_SELECTOR_INL_H_

#include <numeric>
#include <string>
#include <utility>

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
std::optional<DerivedNamedTypeSetT> TypeSelector<DerivedNamedTypeSetT>::from(
    const param::Value& params) {
  // Read the type name from params
  std::string type_name;
  if (params.holds<std::string>()) {
    // If the param is of type string, read the name directly
    type_name = params.get<std::string>().value();
  } else if (params.holds<param::Map>()) {
    // If the param is of type Map, try to read the name from its "type" subkey
    const auto type_param = params.getChild(param::kTypeSelectorKey);
    if (!type_param) {
      LOG(WARNING) << "Nested type name (\"" << param::kTypeSelectorKey
                   << "\") is not set.";
      return std::nullopt;
    }

    if (!type_param->holds<std::string>()) {
      LOG(WARNING) << "Nested type name (\"" << param::kTypeSelectorKey
                   << "\") is not a string.";
      return std::nullopt;
    }

    type_name = type_param->get<std::string>().value();
  } else {
    LOG(WARNING) << "Type names can only be read from params of type string, "
                    "or from param maps (dictionary) with a subkey named \""
                 << param::kTypeSelectorKey << "\".";
    return std::nullopt;
  }

  // Match the type name to a type id
  DerivedNamedTypeSetT type_id(type_name);
  if (!type_id.isValid()) {
    LOG(WARNING)
        << "Type name param (\"" << param::kTypeSelectorKey << ": " << type_name
        << "\") does not match a known type name. Supported type names are ["
        << std::accumulate(std::next(DerivedNamedTypeSetT::names.cbegin()),
                           DerivedNamedTypeSetT::names.cend(),
                           std::string(DerivedNamedTypeSetT::names[0]),
                           [](auto str, const auto& el) -> std::string {
                             return std::move(str) + ", " + std::string(el);
                           })
        << "].";
    return std::nullopt;
  }

  return type_id;
}

template <typename DerivedNamedTypeSetT>
std::optional<DerivedNamedTypeSetT> TypeSelector<DerivedNamedTypeSetT>::from(
    const param::Value& params, const std::string& subconfig_name) {
  if (const auto subconfig_params = params.getChild(subconfig_name);
      subconfig_params) {
    return from(subconfig_params.value());
  }

  LOG(WARNING) << "Tried to load type name from subconfig named "
               << subconfig_name
               << ", but no such subconfig was found. Ignoring request.";
  return std::nullopt;
}
}  // namespace wavemap

#endif  // WAVEMAP_CONFIG_IMPL_TYPE_SELECTOR_INL_H_
