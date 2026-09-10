#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <string>

#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/io/file_conversions.h>
#include <wavemap/layered/classification/local_elevation_classifier.h>

#include "occupancy_map_layers.h"
#include "reflectivity_map_layers.h"
#include "reflectivity_only_map_layers.h"

namespace occupancy_map = wavemap::examples::occupancy_map;
namespace reflectivity_only_map = wavemap::examples::reflectivity_only_map;
namespace reflectivity_map = wavemap::examples::reflectivity_map;

namespace {
struct OccupancyStats {
  size_t blocks = 0u;
  size_t leaves = 0u;
  size_t observed = 0u;
  float min = std::numeric_limits<float>::max();
  float max = std::numeric_limits<float>::lowest();
};

template <typename MapT>
OccupancyStats occupancyStats(const MapT& map) {
  OccupancyStats stats;
  stats.blocks = map.getHashMap().size();
  map.forEachLeaf([&](const wavemap::OctreeIndex&, float occupancy) {
    ++stats.leaves;
    stats.observed += 1e-5f < std::abs(occupancy);
    stats.min = std::min(stats.min, occupancy);
    stats.max = std::max(stats.max, occupancy);
  });
  return stats;
}

struct Comparison {
  size_t samples = 0u;
  size_t above_tolerance = 0u;
  double absolute_error_sum = 0.;
  float max_absolute_error = 0.f;
};

template <typename LhsMapT, typename RhsMapT>
Comparison compareDirection(const LhsMapT& lhs, const RhsMapT& rhs,
                            float tolerance = 1e-4f) {
  Comparison comparison;
  lhs.forEachLeaf([&](const wavemap::OctreeIndex& index, float lhs_value) {
    const float error = std::abs(lhs_value - rhs.getCellValue(index));
    ++comparison.samples;
    comparison.above_tolerance += tolerance < error;
    comparison.absolute_error_sum += error;
    comparison.max_absolute_error =
        std::max(comparison.max_absolute_error, error);
  });
  return comparison;
}

template <typename LhsMapT, typename RhsMapT>
void printComparison(const std::string& name, const LhsMapT& lhs,
                     const RhsMapT& rhs) {
  const Comparison forward = compareDirection(lhs, rhs);
  const Comparison reverse = compareDirection(rhs, lhs);
  const auto print = [](const char* direction, const Comparison& comparison) {
    std::cout << "    " << direction << ": samples=" << comparison.samples
              << " above_1e-4=" << comparison.above_tolerance
              << " mean_abs="
              << (comparison.samples
                      ? comparison.absolute_error_sum / comparison.samples
                      : 0.)
              << " max_abs=" << comparison.max_absolute_error << "\n";
  };
  std::cout << "  " << name << "\n";
  print("forward", forward);
  print("reverse", reverse);
}

template <typename DefinitionT, typename LayerTagT>
void printReflectivityStats(const std::string& name,
                            const typename DefinitionT::Map& map) {
  size_t nonzero = 0u;
  size_t out_of_bounds = 0u;
  float min_value = std::numeric_limits<float>::max();
  float max_value = std::numeric_limits<float>::lowest();
  map.continuousMap().forEachVoxelLeaf(
      [&](const wavemap::OctreeIndex&,
          const typename DefinitionT::Voxel& voxel) {
        const float value = voxel.data.template get<LayerTagT>();
        nonzero += 1e-6f < std::abs(value);
        out_of_bounds += value < 0.f || 1.f < value;
        min_value = std::min(min_value, value);
        max_value = std::max(max_value, value);
      });
  std::cout << "  " << name << ": nonzero=" << nonzero
            << " out_of_[0,1]=" << out_of_bounds
            << " value_range=[" << min_value << ", " << max_value << "]\n";
}

void printOccupancyStats(const std::string& name,
                         const OccupancyStats& stats) {
  std::cout << "  " << name << ": blocks=" << stats.blocks
            << " leaves=" << stats.leaves << " observed=" << stats.observed
            << " range=[" << stats.min << ", " << stats.max << "]\n";
}
}  // namespace

int main(int argc, char** argv) {
  if (argc != 5) {
    std::cerr << "Usage: " << argv[0]
              << " <A.wvmp> <B.lwvmp> <C.lwvmp> <D.lwvmp>\n";
    return 1;
  }

  wavemap::MapBase::Ptr original_base;
  if (!wavemap::io::fileToMap(argv[1], original_base)) {
    std::cerr << "Could not load A\n";
    return 2;
  }
  auto original =
      std::dynamic_pointer_cast<wavemap::HashedWaveletOctree>(original_base);
  if (!original) {
    std::cerr << "A is not a HashedWaveletOctree\n";
    return 2;
  }

  occupancy_map::Definition::Map occupancy(occupancy_map::makeMapConfig());
  reflectivity_only_map::Definition::Map reflectivity(
      reflectivity_only_map::makeMapConfig());
  reflectivity_map::Definition::Map reflectivity_class(
      reflectivity_map::makeMapConfig());
  std::string error;
  if (!occupancy_map::Definition::MapIo::load(argv[2], occupancy, &error)) {
    std::cerr << "Could not load B: " << error << "\n";
    return 2;
  }
  if (!reflectivity_only_map::Definition::MapIo::load(argv[3], reflectivity,
                                                       &error)) {
    std::cerr << "Could not load C: " << error << "\n";
    return 2;
  }
  if (!reflectivity_map::Definition::MapIo::load(argv[4], reflectivity_class,
                                                  &error)) {
    std::cerr << "Could not load D: " << error << "\n";
    return 2;
  }

  std::cout << "Occupancy statistics\n";
  printOccupancyStats("A original", occupancyStats(*original));
  printOccupancyStats("B layered occupancy",
                      occupancyStats(occupancy.continuousMap()));
  printOccupancyStats("C reflectivity",
                      occupancyStats(reflectivity.continuousMap()));
  printOccupancyStats("D reflectivity+class",
                      occupancyStats(reflectivity_class.continuousMap()));

  std::cout << "Occupancy comparisons\n";
  printComparison("A vs B", *original, occupancy.continuousMap());
  printComparison("A vs C", *original, reflectivity.continuousMap());
  printComparison("A vs D", *original, reflectivity_class.continuousMap());
  printComparison("B vs C", occupancy.continuousMap(),
                  reflectivity.continuousMap());
  printComparison("C vs D", reflectivity.continuousMap(),
                  reflectivity_class.continuousMap());

  std::cout << "Reflectivity statistics\n";
  printReflectivityStats<reflectivity_only_map::Definition,
                         reflectivity_only_map::ReflectivityLayer>(
      "C reflectivity", reflectivity);
  printReflectivityStats<reflectivity_map::Definition,
                         reflectivity_map::ReflectivityLayer>(
      "D reflectivity", reflectivity_class);

  const auto& class_layer =
      reflectivity_class.discreteLayers().get<reflectivity_map::ClassLayer>();
  size_t ground = 0u;
  size_t obstacle = 0u;
  for (const auto& [key, cell] : class_layer.cells()) {
    (void)key;
    for (const int offset : cell.observed_offsets) {
      const auto exception = cell.exceptions.find(offset);
      const int value = exception == cell.exceptions.end()
                            ? cell.dominant_value
                            : exception->second;
      ground += value == static_cast<int>(
                             wavemap::layered::GeometricClass::kGround);
      obstacle += value == static_cast<int>(
                               wavemap::layered::GeometricClass::kObstacle);
    }
  }
  const size_t parent_count = class_layer.parentCount();
  const size_t observed_count = class_layer.observedValueCount();
  const size_t exception_count = class_layer.exceptionCount();
  const size_t dominant_matching_count = observed_count - exception_count;
  // Exact size of the version-1 discrete section for this single int layer:
  // section marker, layer metadata/configuration, and all cell records.
  const size_t serialized_discrete_bytes =
      67u + 32u * parent_count + 4u * observed_count +
      8u * exception_count;
  std::cout << "Class statistics\n"
            << "  block_height=" << class_layer.config().block_height
            << " block_side=" << class_layer.config().blockSideLength()
            << " possible_children="
            << class_layer.config().blockVoxelCount() << "\n"
            << "  parents=" << parent_count << " observed=" << observed_count
            << " dominant_matching=" << dominant_matching_count
            << " exceptions=" << exception_count << "\n"
            << "  exception_ratio="
            << (observed_count ? static_cast<double>(exception_count) /
                                     static_cast<double>(observed_count)
                               : 0.)
            << " mean_observed_per_parent="
            << (parent_count ? static_cast<double>(observed_count) /
                                   static_cast<double>(parent_count)
                             : 0.)
            << "\n"
            << "  serialized_discrete_bytes=" << serialized_discrete_bytes
            << " ground=" << ground << " obstacle=" << obstacle << "\n";
  return 0;
}
