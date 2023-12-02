#ifndef WAVEMAP_CONFIG_TYPE_SELECTOR_H_
#define WAVEMAP_CONFIG_TYPE_SELECTOR_H_

#include <string>

#include "wavemap/config/param.h"

namespace wavemap {
namespace param {
static constexpr auto kTypeSelectorKey = "type";
}

template <typename DerivedTypeSelectorT>
struct TypeSelector {
  using TypeId = int;
  using TypeName = std::string;
  static constexpr TypeId kInvalidTypeId = -1;

  // Construct from a given type name or ID
  explicit TypeSelector(const TypeName& type_name) {
    id_ = strToTypeId(type_name);
  }
  TypeSelector(TypeId type_id) {  // NOLINT
    if (isValidTypeId(type_id)) {
      id_ = type_id;
    }
  }

  ~TypeSelector() {
    // Force the derived config classes to specify:
    // - an enum with Ids
    // - an array of names
    // NOTE: These static asserts have to be placed in a method that's
    //       guaranteed to be evaluated by the compiler, making the dtor is a
    //       good option.
    static_assert(std::is_enum_v<typename DerivedTypeSelectorT::Id>,
                  "Derived TypeSelector type must define an enum called Id.");
    static_assert(
        std::is_convertible_v<decltype(DerivedTypeSelectorT::names[0]),
                              std::string>,
        "Derived TypeSelector type must define an array called names whose "
        "members are convertible to std::strings.");
  }

  // Conversions to the underlying type's name or ID
  explicit operator TypeName() const { return typeIdToStr(id_); }
  explicit operator TypeId() const { return id_; }
  TypeName toStr() const { return typeIdToStr(id_); }
  constexpr TypeId toTypeId() const { return id_; }

  // Comparison operators
  friend bool operator==(const DerivedTypeSelectorT& lhs,
                         const DerivedTypeSelectorT& rhs) {
    return lhs.id_ == rhs.id_;
  }
  friend bool operator!=(const DerivedTypeSelectorT& lhs,
                         const DerivedTypeSelectorT& rhs) {
    return !(lhs == rhs);
  }

  // Method to check if the type ID is currently valid
  bool isValid() const { return isValidTypeId(id_); }

  // Static methods for convenience
  // TODO(victorr): Simplify name to "toStr()"
  static TypeName typeIdToStr(TypeId type_id) {
    if (isValidTypeId(type_id)) {
      return DerivedTypeSelectorT::names[static_cast<TypeId>(type_id)];
    } else {
      return "Invalid";
    }
  }
  // TODO(victorr): Simplify name to "toTypeId()"
  static TypeId strToTypeId(const std::string& name);
  // TODO(victorr): Simplify name to "isValid()"
  static bool isValidTypeId(TypeId type_id) {
    return 0 <= type_id &&
           static_cast<size_t>(type_id) < DerivedTypeSelectorT::names.size();
  }

  // Convenience method to read the type from params
  static std::optional<DerivedTypeSelectorT> from(const param::Value& params);
  static std::optional<DerivedTypeSelectorT> from(
      const param::Value& params, const std::string& subconfig_name);

 private:
  TypeId id_ = kInvalidTypeId;

  // Force structs that use TypeSelector (by deriving from it) to pass the right
  // template argument (i.e. themselves) by making the base constructor private
  // and befriending the derived class passed as the template argument
  TypeSelector() = default;
  friend DerivedTypeSelectorT;
};
}  // namespace wavemap

#include "wavemap/config/impl/type_selector_inl.h"

#endif  // WAVEMAP_CONFIG_TYPE_SELECTOR_H_
