#ifndef WAVEMAP_CONFIG_CONFIG_BASE_H_
#define WAVEMAP_CONFIG_CONFIG_BASE_H_

#include "wavemap/config/param.h"
#include "wavemap/config/param_checks.h"
#include "wavemap/config/value_with_unit.h"
#include "wavemap/utils/type_utils.h"

namespace wavemap {
template <typename ConfigDerivedT, size_t num_members = 0,
          typename... CustomMemberTypes>
struct ConfigBase {
  // NOTE: Aggregate initialization will not be available for derived classes
  //       due to the restriction listed on:
  //       https://en.cppreference.com/w/cpp/language/aggregate_initialization
  //       But similar syntax can still be used by manually defining
  //       constructors in the derived class.

  // Force the derived config classes to specify an array describing their
  // members for introspection purposes.
  // NOTE: This static assert has to be placed in a method that's guaranteed
  //       to be evaluated by the compiler, making the dtor is a good option.
  virtual ~ConfigBase() {
    static_assert(0 < num_members,
                  "Derived config type must specify how many members it has by "
                  "setting ConfigBase's num_members template parameter.");
    static_assert(
        std::is_same_v<std::decay_t<decltype(ConfigDerivedT::memberMap[0])>,
                       MemberMetadata>,
        "Derived config type must define a static array called memberMap "
        "describing its member variables.");
    static_assert(std::size(ConfigDerivedT::memberMap) == num_members,
                  "Derived config type's memberMap array must contain one "
                  "MemberMetadata entry for each of its member "
                  "variables.");
  }

  // Setup the introspective member metadata map
  using MemberTypes = param::PrimitiveValueTypes::Append<
      Meters<FloatingPoint>, Radians<FloatingPoint>, Pixels<FloatingPoint>,
      Seconds<FloatingPoint>>::Append<CustomMemberTypes...>;
  using MemberPointer =
      inject_type_list_as_member_ptrs_t<std::variant, ConfigDerivedT,
                                        MemberTypes>;
  struct MemberMetadata {
    param::Name name;
    MemberPointer ptr;
  };
  static constexpr size_t kNumMembers = num_members;
  using MemberMap = const std::array<MemberMetadata, num_members>;

  // Helper methods to check if the config is valid
  virtual bool isValid(bool verbose) const = 0;
  const ConfigDerivedT& checkValid() const {
    CHECK(isValid(true));
    return *static_cast<const ConfigDerivedT*>(this);
  }

  // Load config from param map
  static std::optional<ConfigDerivedT> from(const param::Map& params);

  // Comparison operators
  friend bool operator==(const ConfigDerivedT& lhs, const ConfigDerivedT& rhs) {
    auto is_equal = [&lhs, &rhs](auto&& member_ptr) -> bool {
      return lhs.*member_ptr == rhs.*member_ptr;
    };
    for (const auto& member_metadata : ConfigDerivedT::memberMap) {
      if (!std::visit(is_equal, member_metadata.ptr)) {
        return false;
      }
    }
    return true;
  }
  friend bool operator!=(const ConfigDerivedT& lhs, const ConfigDerivedT& rhs) {
    return !(lhs == rhs);
  }

 private:
  // Force structs that use ConfigBase (by deriving from it) to pass the right
  // template argument (i.e. themselves) by making the base constructor private
  // and befriending the derived class passed as the template argument
  ConfigBase() = default;
  friend ConfigDerivedT;
};
}  // namespace wavemap

#include "wavemap/config/impl/config_base_inl.h"

#endif  // WAVEMAP_CONFIG_CONFIG_BASE_H_
