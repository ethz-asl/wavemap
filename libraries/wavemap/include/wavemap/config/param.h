#ifndef WAVEMAP_CONFIG_PARAM_H_
#define WAVEMAP_CONFIG_PARAM_H_

#include <map>
#include <string>
#include <variant>
#include <vector>

#include "wavemap/common.h"
#include "wavemap/utils/type_utils.h"

namespace wavemap::param {
using Name = std::string;

template <typename... PrimitiveValueTs>
class ValueT {
 public:
  using Array = std::vector<ValueT>;
  using Map = std::map<Name, ValueT>;

  // Constructors
  template <typename T>
  explicit ValueT(T value) : data_(value) {}
  explicit ValueT(double value) : data_(static_cast<FloatingPoint>(value)) {}

  // Methods to check the Value's current type
  template <typename ValueT>
  bool holds() const {
    return std::holds_alternative<ValueT>(data_);
  }

  // Read the value assuming it is of type T. Throws if this is not the case.
  // NOTE: First check if the current value is actually of type T using
  //       holds<T>().
  template <typename T>
  T get() const {
    CHECK(holds<T>());
    return std::get<T>(data_);
  }

 private:
  std::variant<PrimitiveValueTs..., Array, Map> data_;
};

using PrimitiveValueTypes = TypeList<bool, int, FloatingPoint, std::string>;
using Value = inject_type_list_t<ValueT, PrimitiveValueTypes>;
using Array = Value::Array;
using Map = Value::Map;

namespace map {
inline bool hasKey(const Map& map, const Name& key) { return map.count(key); }

template <typename T>
bool keyHoldsValue(const Map& map, const Name& key) {
  return map.count(key) && map.at(key).template holds<T>();
}

template <typename T>
T keyGetValue(const Map& map, const Name& key) {
  return map.at(key).template get<T>();
}

template <typename T>
T keyGetValue(const Map& map, const Name& key, T default_value) {
  if (keyHoldsValue<T>(map, key)) {
    return map.at(key).template get<T>();
  } else {
    return default_value;
  }
}
}  // namespace map
}  // namespace wavemap::param

#endif  // WAVEMAP_CONFIG_PARAM_H_
