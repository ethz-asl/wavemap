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
#include <boost/preprocessor/tuple/size.hpp>

namespace wavemap {
#define MEMBER_NAME_FROM_TUPLE(member_name_tuple) \
  BOOST_PP_TUPLE_ELEM(0, member_name_tuple)

#define MEMBER_UNIT_FROM_TUPLE(member_name_tuple)                  \
  BOOST_PP_EXPR_IIF(                                               \
      BOOST_PP_GREATER(BOOST_PP_TUPLE_SIZE(member_name_tuple), 1), \
      BOOST_PP_TUPLE_ELEM(1, member_name_tuple))

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
  MemberMetadata{BOOST_PP_STRINGIZE(MEMBER_NAME_FROM_TUPLE(member_name_tuple)),                       \
      &class_name::MEMBER_NAME_FROM_TUPLE(member_name_tuple),           \
      MEMBER_UNIT_FROM_TUPLE(member_name_tuple)},

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
// NOTE: We exclude FloatingPoints since these are specialized separately.
template <
    typename ConfigDerivedT, typename MemberPtrT,
    typename ConfigValueT = member_type_t<std::decay_t<MemberPtrT>>,
    std::enable_if_t<param::PrimitiveValueTypes::contains_t<ConfigValueT> &&
                         !std::is_same_v<ConfigValueT, FloatingPoint>,
                     bool> = true>
void loadParam(const param::Name& param_name, const param::Value& param_value,
               ConfigDerivedT& config, MemberPtrT config_member_ptr,
               const std::optional<SiUnit>& /*config_member_unit*/) {
  ConfigValueT& config_value = config.*config_member_ptr;
  if (param_value.holds<ConfigValueT>()) {
    config_value = ConfigValueT{param_value.get<ConfigValueT>()};
  } else {
    LOG(WARNING) << "Type of param " << param_name
                 << " does not match type of corresponding config value.";
  }
}

// Loader for floating point types, which support unit conversions
template <typename ConfigDerivedT>
void loadParam(const param::Name& param_name, const param::Value& param_value,
               ConfigDerivedT& config,
               FloatingPoint ConfigDerivedT::*config_member_ptr,
               const std::optional<SiUnit>& config_member_unit) {
  FloatingPoint& config_value = config.*config_member_ptr;
  if (config_member_unit.has_value()) {
    // TODO(victorr): Extend toUnit to auto-convert ints to floats
    config_value = param::convert::toUnit(
        param_value, config_member_unit.value(), config_value);
  } else {
    if (param_value.holds<FloatingPoint>()) {
      config_value = param_value.get<FloatingPoint>();
    } else if (param_value.holds<int>()) {
      config_value = static_cast<FloatingPoint>(param_value.get<int>());
    } else {
      LOG(WARNING) << "Type of param " << param_name
                   << " does not match type of corresponding config value.";
    }
  }
}

// Loader for types that define a "from" method, such as configs
template <typename ConfigDerivedT, typename MemberPtrT,
          typename ConfigValueT = member_type_t<std::decay_t<MemberPtrT>>,
          decltype(ConfigValueT::from(std::declval<param::Map>()),
                   bool()) = true>
void loadParam(const param::Name& param_name, const param::Value& param_value,
               ConfigDerivedT& config, MemberPtrT config_member_ptr,
               const std::optional<SiUnit>& /*config_member_unit*/) {
  ConfigValueT& config_value = config.*config_member_ptr;
  if (param_value.holds<param::Map>()) {
    config_value = ConfigValueT::from(param_value.get<param::Map>());
  } else {
    LOG(WARNING) << "Type of param " << param_name
                 << " does not match type of corresponding config value.";
  }
}

// Loader for TypeSelector types
template <typename ConfigDerivedT, typename MemberPtrT,
          typename ConfigValueT = member_type_t<std::decay_t<MemberPtrT>>,
          decltype(ConfigValueT::strToTypeId(std::declval<std::string>()),
                   bool()) = true>
void loadParam(const param::Name& param_name, const param::Value& param_value,
               ConfigDerivedT& config, MemberPtrT config_member_ptr,
               const std::optional<SiUnit>& /*config_member_unit*/) {
  ConfigValueT& config_value = config.*config_member_ptr;
  if (param_value.holds<std::string>()) {
    config_value = ConfigValueT::strToTypeId(param_value.get<std::string>());
  } else {
    LOG(WARNING) << "Type of param " << param_name
                 << " does not match type of corresponding config value.";
  }
}
}  // namespace detail

template <typename ConfigDerivedT, size_t num_members,
          typename... CustomMemberTypes>
ConfigDerivedT
ConfigBase<ConfigDerivedT, num_members, CustomMemberTypes...>::from(
    const param::Map& params) {
  ConfigDerivedT config;
  const auto& member_map = ConfigDerivedT::memberMap;

  for (const auto& param_kv : params) {
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
      const auto& unit = member_it->unit;
      auto param_loader = [&](auto&& ptr) {
        detail::loadParam(param_name, param_value, config, ptr, unit);
      };
      std::visit(param_loader, member_it->ptr);
      continue;
    }

    // Issue a warning for unrecognized param names
    LOG(WARNING) << "Ignoring unknown param with name " << param_name;
  }

  return config;
}
}  // namespace wavemap

#endif  // WAVEMAP_CONFIG_IMPL_CONFIG_BASE_INL_H_
