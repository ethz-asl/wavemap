#ifndef WAVEMAP_LAYERED_LOCAL_ELEVATION_CLASSIFIER_H_
#define WAVEMAP_LAYERED_LOCAL_ELEVATION_CLASSIFIER_H_

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <tuple>
#include <type_traits>
#include <vector>

#include <wavemap/core/common.h>
#include <wavemap/core/data_structure/pointcloud.h>
#include <wavemap/layered/integration/endpoint_range.h>
#include <wavemap/layered/integration/layer_observation.h>

namespace wavemap::layered {

enum class GeometricClass : int {
  kUnknown = 0,
  kGround = 1,
  kObstacle = 2,
};

struct LocalElevationClassifierConfig {
  FloatingPoint grid_cell_width = 0.5f;
  FloatingPoint ground_tolerance = 0.2f;
  FloatingPoint obstacle_height = 0.35f;
  size_t min_points_per_cell = 3u;

  bool isValid() const {
    return 0.f < grid_cell_width && 0.f <= ground_tolerance &&
           ground_tolerance < obstacle_height && 0u < min_points_per_cell;
  }
};

// Lightweight frame-local classifier. It estimates the ground elevation as
// the lowest endpoint in each horizontal grid cell. Cells without enough
// support and points in the transition band remain unknown.
class LocalElevationClassifier {
 public:
  explicit LocalElevationClassifier(
      LocalElevationClassifierConfig config = {})
      : config_(config) {
    CHECK(config_.isValid());
  }

  std::vector<GeometricClass> classify(
      const PosedPointcloud<>& pointcloud) const {
    struct CellStats {
      FloatingPoint min_height = std::numeric_limits<FloatingPoint>::max();
      size_t point_count = 0u;
    };

    const FloatingPoint cell_width_inv = 1.f / config_.grid_cell_width;
    std::map<GridCellKey, CellStats> cells;
    std::vector<Point3D> world_points(pointcloud.size());
    std::vector<bool> valid(pointcloud.size(), false);

    for (size_t index = 0u; index < pointcloud.size(); ++index) {
      const Point3D sensor_point = pointcloud[index];
      if (!sensor_point.array().isFinite().all()) {
        continue;
      }
      const Point3D world_point = pointcloud.getPose() * sensor_point;
      if (!world_point.array().isFinite().all()) {
        continue;
      }
      world_points[index] = world_point;
      valid[index] = true;
      CellStats& stats = cells[toCellKey(world_point, cell_width_inv)];
      stats.min_height = std::min(stats.min_height, world_point.z());
      ++stats.point_count;
    }

    std::vector<GeometricClass> classes(pointcloud.size(),
                                        GeometricClass::kUnknown);
    for (size_t index = 0u; index < world_points.size(); ++index) {
      if (!valid[index]) {
        continue;
      }
      const Point3D& point = world_points[index];
      const CellStats& stats = cells.at(toCellKey(point, cell_width_inv));
      if (stats.point_count < config_.min_points_per_cell) {
        continue;
      }
      const FloatingPoint height_above_ground = point.z() - stats.min_height;
      if (height_above_ground <= config_.ground_tolerance) {
        classes[index] = GeometricClass::kGround;
      } else if (config_.obstacle_height <= height_above_ground) {
        classes[index] = GeometricClass::kObstacle;
      }
    }
    return classes;
  }

  const LocalElevationClassifierConfig& config() const { return config_; }

 private:
  struct GridCellKey {
    int x;
    int y;

    bool operator<(const GridCellKey& other) const {
      return std::tie(x, y) < std::tie(other.x, other.y);
    }
  };

  static GridCellKey toCellKey(const Point3D& point,
                               FloatingPoint cell_width_inv) {
    return GridCellKey{
        static_cast<int>(std::floor(point.x() * cell_width_inv)),
        static_cast<int>(std::floor(point.y() * cell_width_inv))};
  }

  LocalElevationClassifierConfig config_;
};

template <typename ValueT>
std::vector<LayerObservation<ValueT>> makeGeometricClassObservations(
    const PosedPointcloud<>& pointcloud,
    const LocalElevationClassifier& classifier,
    const EndpointRange& endpoint_range = {}) {
  static_assert(std::is_integral_v<ValueT>);
  const auto classes = classifier.classify(pointcloud);
  std::vector<LayerObservation<ValueT>> observations;
  observations.reserve(classes.size());
  for (size_t index = 0u; index < classes.size(); ++index) {
    if (classes[index] == GeometricClass::kUnknown ||
        !endpoint_range.contains(pointcloud[index])) {
      continue;
    }
    observations.emplace_back(pointcloud.getPose() * pointcloud[index],
                              static_cast<ValueT>(classes[index]));
  }
  return observations;
}

}  // namespace wavemap::layered

#endif  // WAVEMAP_LAYERED_LOCAL_ELEVATION_CLASSIFIER_H_
