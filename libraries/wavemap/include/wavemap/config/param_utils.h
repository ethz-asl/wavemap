#ifndef WAVEMAP_CONFIG_PARAM_UTILS_H_
#define WAVEMAP_CONFIG_PARAM_UTILS_H_

#include <map>
#include <string>
#include <variant>
#include <vector>

#include "wavemap/common.h"

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
inline FloatingPoint toMeters(const Map& map) {
  if (map::hasKey(map, "meters")) {
    if (map::keyHoldsValue<FloatingPoint>(map, "meters")) {
      return map::keyGetValue<FloatingPoint>(map, "meters");
    } else {
      LOG(ERROR) << "Value held by key meters is not of type FloatingPoint.";
    }
  } else if (map::hasKey(map, "decimeters")) {
    if (map::keyHoldsValue<FloatingPoint>(map, "decimeters")) {
      return 0.1f * map::keyGetValue<FloatingPoint>(map, "decimeters");
    } else {
      LOG(ERROR)
          << "Value held by key decimeters is not of type FloatingPoint.";
    }
  } else if (map::hasKey(map, "centimeters")) {
    if (map::keyHoldsValue<FloatingPoint>(map, "centimeters")) {
      return 0.01f * map::keyGetValue<FloatingPoint>(map, "centimeters");
    } else {
      LOG(ERROR)
          << "Value held by key centimeters is not of type FloatingPoint.";
    }
  } else if (map::hasKey(map, "millimeters")) {
    if (map::keyHoldsValue<FloatingPoint>(map, "millimeters")) {
      return 0.001f * map::keyGetValue<FloatingPoint>(map, "millimeters");
    } else {
      LOG(ERROR)
          << "Value held by key millimeters is not of type FloatingPoint.";
    }
  }
  LOG(FATAL) << "Could not convert value to meters as it contains no sub-key "
                "matching a supported length unit.";
  return kNaN;
}

inline FloatingPoint toRadians(const Map& map) {
  if (map::hasKey(map, "radians")) {
    if (map::keyHoldsValue<FloatingPoint>(map, "radians")) {
      return map::keyGetValue<FloatingPoint>(map, "radians");
    } else {
      LOG(ERROR) << "Value held by key radians is not of type FloatingPoint.";
    }
  } else if (map::hasKey(map, "degrees")) {
    if (map::keyHoldsValue<FloatingPoint>(map, "degrees")) {
      return kPi / 180.f * map::keyGetValue<FloatingPoint>(map, "degrees");
    } else {
      LOG(ERROR) << "Value held by key degrees is not of type FloatingPoint.";
    }
  }
  LOG(FATAL) << "Could not convert value to radians as it contains no sub-key "
                "matching a supported angle unit.";
  return kNaN;
}

inline FloatingPoint toSeconds(const Map& map) {
  if (map::hasKey(map, "seconds")) {
    if (map::keyHoldsValue<FloatingPoint>(map, "seconds")) {
      return map::keyGetValue<FloatingPoint>(map, "seconds");
    } else {
      LOG(ERROR) << "Value held by key seconds is not of type FloatingPoint.";
    }
  }
  LOG(FATAL) << "Could not convert value to seconds as it contains no sub-key "
                "matching a supported time unit.";
  return kNaN;
}

inline FloatingPoint toMeters(const Value& param, FloatingPoint default_value) {
  if (param.holds<Map>()) {
    const auto& sub_map = param.get<Map>();
    return toMeters(sub_map);
  }
  return default_value;
}

inline FloatingPoint toRadians(const Value& param,
                               FloatingPoint default_value) {
  if (param.holds<Map>()) {
    const auto& sub_map = param.get<Map>();
    return toRadians(sub_map);
  }
  return default_value;
}

inline FloatingPoint toSeconds(const Value& param,
                               FloatingPoint default_value) {
  if (param.holds<Map>()) {
    const auto& sub_map = param.get<Map>();
    return toSeconds(sub_map);
  }
  return default_value;
}

inline FloatingPoint toMeters(const Map& map, const Name& key,
                              FloatingPoint default_value) {
  if (map::keyHoldsValue<Map>(map, key)) {
    const auto& sub_map = map::keyGetValue<Map>(map, key);
    return toMeters(sub_map);
  }
  return default_value;
}

inline FloatingPoint toRadians(const Map& map, const Name& key,
                               FloatingPoint default_value) {
  if (map::keyHoldsValue<Map>(map, key)) {
    const auto& sub_map = map::keyGetValue<Map>(map, key);
    return toRadians(sub_map);
  }
  return default_value;
}

inline FloatingPoint toSeconds(const Map& map, const Name& key,
                               FloatingPoint default_value) {
  if (map::keyHoldsValue<Map>(map, key)) {
    const auto& sub_map = map::keyGetValue<Map>(map, key);
    return toSeconds(sub_map);
  }
  return default_value;
}
}  // namespace convert
}  // namespace wavemap::param

#endif  // WAVEMAP_CONFIG_PARAM_UTILS_H_
