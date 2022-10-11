#ifndef WAVEMAP_COMMON_UTILS_CONFIG_UTILS_H_
#define WAVEMAP_COMMON_UTILS_CONFIG_UTILS_H_

#include <map>
#include <string>
#include <variant>
#include <vector>

#include <wavemap_common/common.h>

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

using Value = ValueT<bool, int, FloatingPoint, std::string>;
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

namespace convert {
inline FloatingPoint toMeters(const Value& value) {
  const Map& map = value.get<Map>();
  if (map::hasKey(map, "meters")) {
    return map::keyGetValue<FloatingPoint>(map, "meters");
  }
  LOG(FATAL) << "Could not convert value to meters as it contains no values "
                "with a recognized unit.";
}

inline FloatingPoint toRadians(const Value& value) {
  const Map& map = value.get<Map>();
  if (map::hasKey(map, "radians")) {
    return map::keyGetValue<FloatingPoint>(map, "radians");
  } else if (map::hasKey(map, "degrees")) {
    return kPi / 180.f * map::keyGetValue<FloatingPoint>(map, "degrees");
  }
  LOG(FATAL) << "Could not convert value to radians as it contains no values "
                "with a recognized unit.";
}
}  // namespace convert
}  // namespace wavemap::param

#endif  // WAVEMAP_COMMON_UTILS_CONFIG_UTILS_H_
