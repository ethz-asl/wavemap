#ifndef WAVEMAP_COMMON_UTILS_FACTORY_UTILS_H_
#define WAVEMAP_COMMON_UTILS_FACTORY_UTILS_H_

#include <string>

namespace wavemap {
template <typename DerivedNamedTypeSetT>
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

  // Conversions to the underlying type's name or ID
  explicit operator TypeName() const { return typeIdToStr(id_); }
  explicit operator TypeId() const { return id_; }
  TypeName toStr() const { return typeIdToStr(id_); }
  TypeId toTypeId() const { return id_; }

  // Method to check if the type ID is currently valid
  bool isValid() const { return isValidTypeId(id_); }

  // Static methods for convenience
  static TypeName typeIdToStr(TypeId intersection_type) {
    return DerivedNamedTypeSetT::names[static_cast<TypeId>(intersection_type)];
  }
  static TypeId strToTypeId(const std::string& name);
  static bool isValidTypeId(TypeId type_id) {
    return 0 <= type_id &&
           static_cast<size_t>(type_id) < DerivedNamedTypeSetT::names.size();
  }

  // Convenience method to read the type from a param map
  static DerivedNamedTypeSetT fromParamMap(const param::Map& params,
                                           std::string& error_msg);

 private:
  TypeId id_ = kInvalidTypeId;
};

}  // namespace wavemap

#include "wavemap_common/utils/impl/factory_utils_inl.h"

#endif  // WAVEMAP_COMMON_UTILS_FACTORY_UTILS_H_
