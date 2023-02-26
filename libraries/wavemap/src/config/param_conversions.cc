#include "wavemap/config/param_conversions.h"

namespace wavemap::param::convert {
FloatingPoint toMeters(const Map& map) {
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

FloatingPoint toRadians(const Map& map) {
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

FloatingPoint toSeconds(const Map& map) {
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

FloatingPoint toMeters(const Value& param, FloatingPoint default_value) {
  if (param.holds<Map>()) {
    const auto& sub_map = param.get<Map>();
    return toMeters(sub_map);
  }
  return default_value;
}

FloatingPoint toRadians(const Value& param, FloatingPoint default_value) {
  if (param.holds<Map>()) {
    const auto& sub_map = param.get<Map>();
    return toRadians(sub_map);
  }
  return default_value;
}

FloatingPoint toSeconds(const Value& param, FloatingPoint default_value) {
  if (param.holds<Map>()) {
    const auto& sub_map = param.get<Map>();
    return toSeconds(sub_map);
  }
  return default_value;
}

FloatingPoint toMeters(const Map& map, const Name& key,
                       FloatingPoint default_value) {
  if (map::keyHoldsValue<Map>(map, key)) {
    const auto& sub_map = map::keyGetValue<Map>(map, key);
    return toMeters(sub_map);
  }
  return default_value;
}

FloatingPoint toRadians(const Map& map, const Name& key,
                        FloatingPoint default_value) {
  if (map::keyHoldsValue<Map>(map, key)) {
    const auto& sub_map = map::keyGetValue<Map>(map, key);
    return toRadians(sub_map);
  }
  return default_value;
}

FloatingPoint toSeconds(const Map& map, const Name& key,
                        FloatingPoint default_value) {
  if (map::keyHoldsValue<Map>(map, key)) {
    const auto& sub_map = map::keyGetValue<Map>(map, key);
    return toSeconds(sub_map);
  }
  return default_value;
}
}  // namespace wavemap::param::convert
