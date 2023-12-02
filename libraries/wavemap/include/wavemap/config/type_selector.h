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
  TypeSelector(const TypeName& type_name);  // NOLINT
  TypeSelector(TypeId type_id);             // NOLINT

  // Destructor
  ~TypeSelector();

  // Allow implicit conversions to the underlying type's name or ID
  operator TypeName() const { return toStr(id_); }
  operator TypeId() const { return id_; }

  // Method to check if the type ID is currently valid
  bool isValid() const { return isValid(id_); }
  static bool isValid(TypeId type_id);

  // Explicit conversions to the underlying type's name or ID
  TypeName toStr() const { return toStr(id_); }
  static TypeName toStr(TypeId type_id);
  constexpr TypeId toTypeId() const { return id_; }
  static TypeId toTypeId(const std::string& name);

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
