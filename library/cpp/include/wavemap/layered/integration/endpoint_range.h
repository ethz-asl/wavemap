#ifndef WAVEMAP_LAYERED_ENDPOINT_RANGE_H_
#define WAVEMAP_LAYERED_ENDPOINT_RANGE_H_

#include <cmath>
#include <limits>

#include <wavemap/core/common.h>

namespace wavemap::layered {

struct EndpointRange {
  FloatingPoint min = 0.f;
  FloatingPoint max = std::numeric_limits<FloatingPoint>::infinity();

  bool isValid() const {
    return 0.f <= min && min <= max && !std::isnan(max);
  }

  bool contains(const Point3D& sensor_point) const {
    if (!sensor_point.array().isFinite().all()) {
      return false;
    }
    const FloatingPoint squared_range = sensor_point.squaredNorm();
    return min * min <= squared_range && squared_range <= max * max;
  }
};

}  // namespace wavemap::layered

#endif  // WAVEMAP_LAYERED_ENDPOINT_RANGE_H_
