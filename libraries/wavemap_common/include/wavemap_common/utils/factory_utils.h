#ifndef WAVEMAP_COMMON_UTILS_FACTORY_UTILS_H_
#define WAVEMAP_COMMON_UTILS_FACTORY_UTILS_H_

#include <string>

namespace wavemap {
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
    // NOTE: These static assert have to be placed in a method that's guaranteed
    //       to be evaluated by the compiler, making the dtor is a good option.
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
  TypeId toTypeId() const { return id_; }

  // Comparison operator for convenience
  bool operator==(TypeId rhs) { return id_ == rhs; }

  // Method to check if the type ID is currently valid
  bool isValid() const { return isValidTypeId(id_); }

  // Static methods for convenience
  static TypeName typeIdToStr(TypeId update_type) {
    return DerivedTypeSelectorT::names[static_cast<TypeId>(update_type)];
  }
  static TypeId strToTypeId(const std::string& name);
  static bool isValidTypeId(TypeId type_id) {
    return 0 <= type_id &&
           static_cast<size_t>(type_id) < DerivedTypeSelectorT::names.size();
  }

  // Convenience method to read the type from a param map
  static DerivedTypeSelectorT fromParamMap(const param::Map& params,
                                           std::string& error_msg);

 private:
  TypeId id_ = kInvalidTypeId;

  // Force structs that use TypeSelector (by deriving from it) to pass the right
  // template argument (i.e. themselves) by making the base constructor private
  // and befriending the derived class passed as the template argument
  TypeSelector() = default;
  friend DerivedTypeSelectorT;
};

}  // namespace wavemap

#include "wavemap_common/utils/impl/factory_utils_inl.h"

#endif  // WAVEMAP_COMMON_UTILS_FACTORY_UTILS_H_
