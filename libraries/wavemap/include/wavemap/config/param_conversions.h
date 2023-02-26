#ifndef WAVEMAP_CONFIG_PARAM_CONVERSIONS_H_
#define WAVEMAP_CONFIG_PARAM_CONVERSIONS_H_

#include "wavemap/common.h"
#include "wavemap/config/param.h"

namespace wavemap::param::convert {
FloatingPoint toMeters(const Map& map);

FloatingPoint toRadians(const Map& map);

FloatingPoint toSeconds(const Map& map);

FloatingPoint toMeters(const Value& param, FloatingPoint default_value);

FloatingPoint toRadians(const Value& param, FloatingPoint default_value);

FloatingPoint toSeconds(const Value& param, FloatingPoint default_value);

FloatingPoint toMeters(const Map& map, const Name& key,
                       FloatingPoint default_value);

FloatingPoint toRadians(const Map& map, const Name& key,
                        FloatingPoint default_value);

FloatingPoint toSeconds(const Map& map, const Name& key,
                        FloatingPoint default_value);
}  // namespace wavemap::param::convert

#endif  // WAVEMAP_CONFIG_PARAM_CONVERSIONS_H_
