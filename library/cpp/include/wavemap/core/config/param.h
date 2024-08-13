#ifndef WAVEMAP_CORE_CONFIG_PARAM_H_
#define WAVEMAP_CORE_CONFIG_PARAM_H_

#include <map>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "wavemap/core/common.h"
#include "wavemap/core/utils/meta/type_utils.h"

namespace wavemap::param {
// Type alias for param::Map keys
using Name = std::string;

// Template for a param value that can hold either
// * one of the primitive types specified through PrimitiveValueTs
// * an array (list) of param values
// * a map (dictionary) of param values, indexed by keys of type param::Name
// NOTE: This recursion allows one param value to hold a tree of parameters.
template <typename... PrimitiveValueTs>
class ValueT {
 public:
  using Array = std::vector<ValueT>;
  using Map = std::map<Name, ValueT>;

  // Constructors
  template <typename T>
  explicit ValueT(T value) : data_(value) {}
  explicit ValueT(double value) : data_(static_cast<FloatingPoint>(value)) {}

  // Method to change the value's current type, possibly emplacing a new value
  template <class T, class... Args>
  T& emplace(Args&&... args) {
    return data_.template emplace<T>(std::forward<Args>(args)...);
  }

  // Methods to check the Value's current type
  template <typename ValueT>
  bool holds() const {
    return std::holds_alternative<ValueT>(data_);
  }

  // Try to read the value using type T. Returns an empty optional if it fails.
  template <typename T>
  std::optional<T> as() const {
    if (holds<T>()) {
      return std::get<T>(data_);
    } else {
      return std::nullopt;
    }
  }

  // Methods to work with nested configs
  inline bool hasChild(const Name& key) const {
    if (const auto map = as<Map>(); map) {
      return map.value().count(key);
    }
    return false;
  }
  std::optional<ValueT> getChild(const Name& key) const {
    if (const auto map = as<Map>(); map) {
      if (const auto it = map.value().find(key); it != map.value().end()) {
        return it->second;
      }
    }
    return std::nullopt;
  }
  template <typename T>
  std::optional<T> getChildAs(const Name& key) const {
    if (const auto child = getChild(key); child) {
      return child.value().template as<T>();
    }
    return std::nullopt;
  }

 private:
  std::variant<PrimitiveValueTs..., Array, Map> data_;
};

// Default primitive types that a param value can hold
using PrimitiveValueTypes =
    meta::TypeList<bool, int, FloatingPoint, std::string>;

// Instantiate a param::Value type that can hold our default primitive types
using Value = meta::inject_type_list_t<ValueT, PrimitiveValueTypes>;

// Lift value Array and Map types into param:: namespace
using Array = Value::Array;
using Map = Value::Map;
}  // namespace wavemap::param

#endif  // WAVEMAP_CORE_CONFIG_PARAM_H_
