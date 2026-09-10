#ifndef WAVEMAP_LAYERED_POINTCLOUD_LAYER_INTEGRATION_MODE_H_
#define WAVEMAP_LAYERED_POINTCLOUD_LAYER_INTEGRATION_MODE_H_

#include <utility>

#include <wavemap/core/indexing/index_conversions.h>
#include <wavemap/core/utils/iterate/ray_iterator.h>

namespace wavemap::layered {

// Where a value attached to a PointCloud2 sample is written. EndpointOnly is
// the appropriate default for surface measurements such as reflectivity,
// color, roughness, and class. AlongRay is intentionally opt-in because it is
// both more expensive and only meaningful for values that describe traversed
// free space.
enum class PointcloudLayerIntegrationMode { kEndpointOnly, kAlongRay };

template <PointcloudLayerIntegrationMode Mode, typename VisitorT>
void forEachPointcloudIntegrationIndex(const Point3D& sensor_origin,
                                       const Point3D& endpoint,
                                       FloatingPoint cell_width,
                                       VisitorT&& visitor) {
  static_assert(Mode == PointcloudLayerIntegrationMode::kEndpointOnly ||
                    Mode == PointcloudLayerIntegrationMode::kAlongRay,
                "Unsupported pointcloud layer integration mode.");
  if constexpr (Mode == PointcloudLayerIntegrationMode::kEndpointOnly) {
    visitor(convert::pointToNearestIndex(endpoint, 1.f / cell_width));
  } else {
    for (const Index3D& index : Ray<3>(sensor_origin, endpoint, cell_width)) {
      visitor(index);
    }
  }
}

}  // namespace wavemap::layered

#endif  // WAVEMAP_LAYERED_POINTCLOUD_LAYER_INTEGRATION_MODE_H_
