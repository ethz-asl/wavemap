#ifndef WAVEMAP_CORE_CONFIG_IMPL_TYPE_SELECTOR_INL_H_
#define WAVEMAP_CORE_CONFIG_IMPL_TYPE_SELECTOR_INL_H_

#include <numeric>
#include <string>
#include <utility>

#include "wavemap/core/utils/print/container.h"

namespace wavemap {
inline std::optional<std::string> param::getTypeStr(
    const param::Value& params) {
  // If the param is of type string, read the name directly
  if (params.holds<std::string>()) {
    return params.as<std::string>().value();
  }

  // If the param is of type Map, try to read the name from its "type" subkey
  if (params.holds<param::Map>()) {
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

    return type_param->as<std::string>().value();
  }

  LOG(WARNING) << "Type names can only be read from params of type string, "
                  "or from param maps (dictionary) with a subkey named \""
               << param::kTypeSelectorKey << "\".";
  return std::nullopt;
}

template <typename DerivedTypeSelectorT>
TypeSelector<DerivedTypeSelectorT>::TypeSelector(
    const TypeSelector::TypeName& type_name) {
  id_ = toTypeId(type_name);
}

template <typename DerivedTypeSelectorT>
TypeSelector<DerivedTypeSelectorT>::TypeSelector(TypeId type_id) {
  if (isValid(type_id)) {
    id_ = type_id;
  }
}

template <typename DerivedTypeSelectorT>
TypeSelector<DerivedTypeSelectorT>::~TypeSelector() {
  // Force the derived config classes to specify:
  // - an enum with Ids, using type "int" as the enum's underlying type
  // - an array of names
  // NOTE: These static asserts have to be placed in a method that's
  //       guaranteed to be evaluated by the compiler, making the dtor is a
  //       good option.
  static_assert(
      std::is_same_v<std::underlying_type_t<typename DerivedTypeSelectorT::Id>,
                     int>);
  static_assert(std::is_enum_v<typename DerivedTypeSelectorT::Id>,
                "Derived TypeSelector type must define an enum called Id.");
  static_assert(
      std::is_convertible_v<decltype(DerivedTypeSelectorT::names[0]),
                            std::string>,
      "Derived TypeSelector type must define an array called names whose "
      "members are convertible to std::strings.");
}

template <typename DerivedTypeSelectorT>
bool TypeSelector<DerivedTypeSelectorT>::isValid(TypeId type_id) {
  return 0 <= type_id &&
         static_cast<size_t>(type_id) < DerivedTypeSelectorT::names.size();
}

template <typename DerivedTypeSelectorT>
typename TypeSelector<DerivedTypeSelectorT>::TypeName
TypeSelector<DerivedTypeSelectorT>::toStr(TypeId type_id) {
  if (isValid(type_id)) {
    return DerivedTypeSelectorT::names[static_cast<TypeId>(type_id)];
  } else {
    return "Invalid";
  }
}

template <typename DerivedTypeSelectorT>
typename TypeSelector<DerivedTypeSelectorT>::TypeId
TypeSelector<DerivedTypeSelectorT>::toTypeId(const std::string& name) {
  for (size_t type_idx = 0; type_idx < DerivedTypeSelectorT::names.size();
       ++type_idx) {
    if (name == DerivedTypeSelectorT::names[type_idx]) {
      return static_cast<TypeId>(type_idx);
    }
  }
  return kInvalidTypeId;
}

template <typename DerivedTypeSelectorT>
std::optional<DerivedTypeSelectorT> TypeSelector<DerivedTypeSelectorT>::from(
    const std::string& type_name) {
  DerivedTypeSelectorT type_id(type_name);
  if (!type_id.isValid()) {
    LOG(WARNING)
        << "Value of type name param \"" << param::kTypeSelectorKey << "\": \""
        << type_name
        << "\" does not match a known type name. Supported type names are ["
        << print::sequence(DerivedTypeSelectorT::names) << "].";
    return std::nullopt;
  }

  return type_id;
}

template <typename DerivedTypeSelectorT>
std::optional<DerivedTypeSelectorT> TypeSelector<DerivedTypeSelectorT>::from(
    const param::Value& params) {
  // Read the type name from params
  const auto type_name = param::getTypeStr(params);
  if (!type_name) {
    // No type name was defined
    // NOTE: A message explaining the failure is already printed by getTypeStr.
    return std::nullopt;
  }

  // Match the type name to a type id
  return from(type_name.value());
}

template <typename DerivedTypeSelectorT>
std::optional<DerivedTypeSelectorT> TypeSelector<DerivedTypeSelectorT>::from(
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

#endif  // WAVEMAP_CORE_CONFIG_IMPL_TYPE_SELECTOR_INL_H_
