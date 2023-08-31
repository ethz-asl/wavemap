#ifndef WAVEMAP_CONFIG_IMPL_CONFIG_BASE_INL_H_
#define WAVEMAP_CONFIG_IMPL_CONFIG_BASE_INL_H_

#include <string>

#include <boost/preprocessor/comma_if.hpp>
#include <boost/preprocessor/comparison/greater.hpp>
#include <boost/preprocessor/control/expr_iif.hpp>
#include <boost/preprocessor/facilities/empty.hpp>
#include <boost/preprocessor/seq/for_each.hpp>
#include <boost/preprocessor/seq/size.hpp>
#include <boost/preprocessor/seq/variadic_seq_to_seq.hpp>
#include <boost/preprocessor/stringize.hpp>
#include <boost/preprocessor/tuple/elem.hpp>
#include <boost/preprocessor/tuple/rem.hpp>
#include <boost/preprocessor/tuple/size.hpp>

namespace wavemap {
#define MEMBER_NAME_FROM_TUPLE(member_name_tuple) \
  BOOST_PP_TUPLE_ELEM(0, member_name_tuple)

#define ASSERT_CONFIG_MEMBER_TYPE_IS_SUPPORTED(r, class_name,      \
                                               member_name_tuple)  \
  static_assert(                                                   \
      class_name::MemberTypes::contains_t<decltype(                \
          class_name::MEMBER_NAME_FROM_TUPLE(member_name_tuple))>, \
      BOOST_PP_STRINGIZE(                                                   \
          The type of class_name::MEMBER_NAME_FROM_TUPLE(member_name_tuple) \
          is not supported by default and has not been announced as a       \
          CustomMemberType. Make sure to include this custom type in the    \
          CustomMemberTypes parameter pack passed to ConfigBase.));

#define ASSERT_ALL_CONFIG_MEMBERS_DECLARED(class_name, member_sequence)       \
  static_assert(                                                              \
      class_name::kNumMembers ==                                              \
          BOOST_PP_SEQ_SIZE(BOOST_PP_VARIADIC_SEQ_TO_SEQ(member_sequence)),   \
      "The number of config members declared through DECLARE_CONFIG_MEMBERS " \
      "must match the number of members announced through the num_members "   \
      "template argument passed to ConfigBase.");

#define ASSERT_ALL_CONFIG_MEMBER_TYPES_SUPPORTED(class_name, member_sequence) \
  BOOST_PP_SEQ_FOR_EACH(ASSERT_CONFIG_MEMBER_TYPE_IS_SUPPORTED, class_name,   \
                        BOOST_PP_VARIADIC_SEQ_TO_SEQ(member_sequence))

#define APPEND_CONFIG_MEMBER_METADATA(r, class_name, member_name_tuple) \
  MemberMetadata{                                                       \
      BOOST_PP_STRINGIZE(MEMBER_NAME_FROM_TUPLE(member_name_tuple)),    \
                         &class_name::BOOST_PP_TUPLE_REM() member_name_tuple},

#define GENERATE_CONFIG_MEMBER_MAP(class_name, member_sequence)          \
  class_name::MemberMap class_name::memberMap {                          \
    BOOST_PP_SEQ_FOR_EACH(APPEND_CONFIG_MEMBER_METADATA, class_name,     \
                          BOOST_PP_VARIADIC_SEQ_TO_SEQ(member_sequence)) \
  }

#define DECLARE_CONFIG_MEMBERS(class_name, member_sequence)             \
  ASSERT_ALL_CONFIG_MEMBERS_DECLARED(class_name, member_sequence)       \
  ASSERT_ALL_CONFIG_MEMBER_TYPES_SUPPORTED(class_name, member_sequence) \
  GENERATE_CONFIG_MEMBER_MAP(class_name, member_sequence)

namespace detail {
// Loader for PrimitiveValueTypes
template <typename ConfigDerivedT, typename MemberPtrT,
          typename ConfigValueT = member_type_t<std::decay_t<MemberPtrT>>,
          std::enable_if_t<param::PrimitiveValueTypes::contains_t<ConfigValueT>,
                           bool> = true>
void loadParam(const param::Name& param_name, const param::Value& param_value,
               ConfigDerivedT& config, MemberPtrT config_member_ptr) {
  ConfigValueT& config_value = config.*config_member_ptr;
  if (param_value.holds<ConfigValueT>()) {
    config_value = ConfigValueT{param_value.get<ConfigValueT>().value()};
    return;
  } else if constexpr (std::is_same_v<ConfigValueT, FloatingPoint>) {
    if (const auto param_int = param_value.get<int>(); param_int) {
      // If the param_value and config_value's types do not match exactly, we
      // still allow automatic conversions from ints to floats
      // NOTE: This is avoids pesky, potentially confusing errors when setting
      //       whole numbers and forgetting the decimal point (e.g. 1 vs 1.0).
      config_value =
          ConfigValueT{static_cast<FloatingPoint>(param_int.value())};
      return;
    }
  }

  LOG(WARNING) << "Type of param " << param_name
               << " does not match the type of its corresponding config value. "
                  "Keeping default value \""
               << config_value << "\".";
}

// Loader for types that define a "from" method, such as configs derived from
// ConfigBase, type IDs derived from TypeSelector, and config values derived
// from ValueWithUnits
template <typename ConfigDerivedT, typename MemberPtrT,
          typename ConfigValueT = member_type_t<std::decay_t<MemberPtrT>>,
          decltype(ConfigValueT::from(std::declval<param::Value>()),
                   bool()) = true>
void loadParam(const param::Name& param_name, const param::Value& param_value,
               ConfigDerivedT& config, MemberPtrT config_member_ptr) {
  ConfigValueT& config_value = config.*config_member_ptr;
  if (const auto read_value = ConfigValueT::from(param_value); read_value) {
    config_value = read_value.value();
  } else {
    // Report the error, and print the fallback (default) value that will be
    // used instead if possible
    if constexpr (has_to_str_member_fn_v<ConfigValueT>) {
      LOG(WARNING) << "Param " << param_name
                   << " could not be loaded. Keeping default value: "
                   << config_value.toStr() << ".";
    } else {
      LOG(WARNING) << "Param " << param_name
                   << " could not be loaded. Keeping default value.";
    }
  }
}
}  // namespace detail

template <typename ConfigDerivedT, size_t num_members,
          typename... CustomMemberTypes>
std::optional<ConfigDerivedT>
ConfigBase<ConfigDerivedT, num_members, CustomMemberTypes...>::from(
    const param::Value& params) {
  const auto param_map = params.get<param::Map>();
  if (!param_map) {
    LOG(WARNING) << "Tried to load config from a param that is not of type Map "
                    "(dictionary). Will be ignored.";
    return std::nullopt;
  }

  ConfigDerivedT config;
  const auto& member_map = ConfigDerivedT::memberMap;

  for (const auto& param_kv : param_map.value()) {
    const auto& param_name = param_kv.first;
    const auto& param_value = param_kv.second;

    // Skip keys used for TypeSelectors
    // NOTE: These keys are used by the factory methods to decide which module
    //       (and config) type should be instantiated. They do not refer to
    //       actual members of the configs themselves and can thus be ignored in
    //       the current method.
    if (param_name == param::kTypeSelectorKey) {
      continue;
    }

    // See if the current param name matches one of the config's members
    const auto& member_it = std::find_if(
        member_map.begin(), member_map.end(),
        [&](const auto& member) { return member.name == param_name; });

    // If so, load it
    if (member_it != member_map.end()) {
      auto param_loader = [&](auto&& ptr) {
        detail::loadParam(param_name, param_value, config, ptr);
      };
      std::visit(param_loader, member_it->ptr);
      continue;
    }

    // Issue a warning for unrecognized param names
    LOG(WARNING) << "Ignoring unknown param with name " << param_name;
  }

  return config;
}

template <typename ConfigDerivedT, size_t num_members,
          typename... CustomMemberTypes>
std::optional<ConfigDerivedT>
ConfigBase<ConfigDerivedT, num_members, CustomMemberTypes...>::from(
    const param::Value& params, const std::string& subconfig_name) {
  if (const auto subconfig_params = params.getChild(subconfig_name);
      subconfig_params) {
    return from(subconfig_params.value());
  }

  LOG(WARNING) << "Tried to load subconfig named " << subconfig_name
               << ", but params contained no such key. Ignoring request.";
  return std::nullopt;
}
}  // namespace wavemap

#endif  // WAVEMAP_CONFIG_IMPL_CONFIG_BASE_INL_H_
