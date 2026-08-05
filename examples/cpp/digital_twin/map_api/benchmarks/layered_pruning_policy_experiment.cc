#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "../../common/layered_voxel_config.h"

struct Sample {
  wavemap::Index3D index;
  LayeredVoxel expected;
};

struct ErrorStats {
  float occupancy_sum = 0.f;
  float rgb_sum = 0.f;
  float traversability_sum = 0.f;
  float max_occupancy = 0.f;
  float max_rgb = 0.f;
  float max_traversability = 0.f;
};

struct PruningRunStats {
  std::string label;
  std::string dataset;
  size_t nodes_before = 0u;
  size_t nodes_after = 0u;
  size_t memory_before = 0u;
  size_t memory_after = 0u;
  long long write_us = 0;
  long long read_before_us = 0;
  long long threshold_prune_us = 0;
  ErrorStats before_expected;
  ErrorStats after_expected;
  ErrorStats prune_delta;
};

LayeredVoxel makeLargeExpectedVoxel(int x, int y, int z) {
  const wavemap::FloatingPoint occupancy = 1.f;
  const Rgb color = rgb(static_cast<float>(x) / 49.f,
                        static_cast<float>(y) / 49.f,
                        static_cast<float>(z) / 39.f);
  const float traversability =
      static_cast<float>((x + 2 * y + 3 * z) % 97) / 96.f;
  return makeLayeredVoxel(occupancy, color, traversability);
}

std::vector<Sample> makeLargeSamples() {
  constexpr int kSizeX = 50;
  constexpr int kSizeY = 50;
  constexpr int kSizeZ = 40;

  std::vector<Sample> samples;
  samples.reserve(kSizeX * kSizeY * kSizeZ);
  for (int x = 0; x < kSizeX; ++x) {
    for (int y = 0; y < kSizeY; ++y) {
      for (int z = 0; z < kSizeZ; ++z) {
        const wavemap::Index3D index(x, y, z);
        samples.push_back({index, makeLargeExpectedVoxel(x, y, z)});
      }
    }
  }
  return samples;
}

LayeredVoxel makeLayerBoundaryExpectedVoxel(int x, int y, int z) {
  const wavemap::FloatingPoint occupancy = 0.5f;

  Rgb color = rgb(0.2f, 0.2f, 0.2f);
  if (16 <= x) {
    color = rgb(0.9f, 0.1f, 0.1f);
  }
  if (16 <= y) {
    color = rgb(0.1f, 0.1f, 0.9f);
  }

  float traversability = 0.25f;
  if (8 <= z && z < 24) {
    traversability = 0.85f;
  }

  return makeLayeredVoxel(occupancy, color, traversability);
}

std::vector<Sample> makeLayerBoundarySamples() {
  constexpr int kSize = 32;

  std::vector<Sample> samples;
  samples.reserve(kSize * kSize * kSize);
  for (int x = 0; x < kSize; ++x) {
    for (int y = 0; y < kSize; ++y) {
      for (int z = 0; z < kSize; ++z) {
        const wavemap::Index3D index(x, y, z);
        samples.push_back({index, makeLayerBoundaryExpectedVoxel(x, y, z)});
      }
    }
  }
  return samples;
}

float rgbError(const LayeredVoxel& read, const LayeredVoxel& expected) {
  return sumAbsDiff(read.data.rgb, expected.data.rgb);
}

float traversabilityError(const LayeredVoxel& read,
                          const LayeredVoxel& expected) {
  return std::abs(read.data.traversability - expected.data.traversability);
}

float occupancyError(const LayeredVoxel& read, const LayeredVoxel& expected) {
  return std::abs(read.occupancy - expected.occupancy);
}

ErrorStats computeErrorsAgainstExpected(const ContinuousWaveletMap& map,
                                        const std::vector<Sample>& samples) {
  ErrorStats stats;
  for (const auto& sample : samples) {
    const LayeredVoxel read = map.getVoxelValue(sample.index);
    const float occ_error = occupancyError(read, sample.expected);
    const float color_error = rgbError(read, sample.expected);
    const float trav_error = traversabilityError(read, sample.expected);
    stats.occupancy_sum += occ_error;
    stats.rgb_sum += color_error;
    stats.traversability_sum += trav_error;
    stats.max_occupancy = std::max(stats.max_occupancy, occ_error);
    stats.max_rgb = std::max(stats.max_rgb, color_error);
    stats.max_traversability = std::max(stats.max_traversability, trav_error);
  }
  return stats;
}

ErrorStats computeErrorsAgainstBaseline(
    const ContinuousWaveletMap& map, const std::vector<Sample>& samples,
    const std::vector<LayeredVoxel>& baseline) {
  ErrorStats stats;
  for (size_t sample_idx = 0; sample_idx < samples.size(); ++sample_idx) {
    const LayeredVoxel read = map.getVoxelValue(samples[sample_idx].index);
    const LayeredVoxel& expected = baseline[sample_idx];
    const float occ_error = occupancyError(read, expected);
    const float color_error = rgbError(read, expected);
    const float trav_error = traversabilityError(read, expected);
    stats.occupancy_sum += occ_error;
    stats.rgb_sum += color_error;
    stats.traversability_sum += trav_error;
    stats.max_occupancy = std::max(stats.max_occupancy, occ_error);
    stats.max_rgb = std::max(stats.max_rgb, color_error);
    stats.max_traversability = std::max(stats.max_traversability, trav_error);
  }
  return stats;
}

std::vector<LayeredVoxel> readBackAll(const ContinuousWaveletMap& map,
                                      const std::vector<Sample>& samples) {
  std::vector<LayeredVoxel> values;
  values.reserve(samples.size());
  for (const auto& sample : samples) {
    values.push_back(map.getVoxelValue(sample.index));
  }
  return values;
}

bool detailsNonzero(const LayeredBlock::Coefficients::Details& details,
                    const ContinuousWaveletMap::PruningConfig& config) {
  return std::any_of(details.cbegin(), details.cend(),
                     [&config](const LayeredVoxel& voxel) {
                       return wavemap::CellDataTraits<LayeredVoxel>::isNonzero(
                           voxel, config);
                     });
}

float detailScore(const LayeredVoxel& voxel,
                  const ContinuousWaveletMap::PruningConfig& config) {
  return wavemap::CellDataTraits<LayeredVoxel>::pruningScore(voxel, config);
}

void printCoefficientPolicyCheck() {
  LayeredBlock::Coefficients::Details zero_details{};
  LayeredBlock::Coefficients::Details occupancy_details{};
  LayeredBlock::Coefficients::Details color_details{};
  LayeredBlock::Coefficients::Details combined_small_details{};

  occupancy_details[0] = makeLayeredVoxel(0.25f, rgb(0.f, 0.f, 0.f), 0.f);
  color_details[0] = makeLayeredVoxel(0.f, rgb(1.f, 0.f, 0.f), 0.5f);
  combined_small_details[0] =
      makeLayeredVoxel(0.0004f, rgb(0.0004f, 0.f, 0.f), 0.0004f);

  const auto occupancy_only = makeLayeredPruningConfig(1.f, 0.f, 0.f);
  const auto equal_layers = makeLayeredPruningConfig(1.f, 1.f, 1.f);
  const auto traversability_heavy = makeLayeredPruningConfig(1.f, 1.f, 2.f);

  const auto print_case = [](const char* label,
                             const LayeredBlock::Coefficients::Details& details,
                             const ContinuousWaveletMap::PruningConfig& config) {
    std::cout << "    " << label << ": nonzero="
              << detailsNonzero(details, config)
              << " score[0]=" << detailScore(details[0], config) << "\n";
  };

  std::cout << "Direct coefficient pruning check\n";
  std::cout << "  occupancy-only config\n";
  print_case("zero", zero_details, occupancy_only);
  print_case("occupancy detail", occupancy_details, occupancy_only);
  print_case("color/traversability detail", color_details, occupancy_only);
  print_case("combined small details", combined_small_details,
             occupancy_only);

  std::cout << "  equal-layer config\n";
  print_case("zero", zero_details, equal_layers);
  print_case("occupancy detail", occupancy_details, equal_layers);
  print_case("color/traversability detail", color_details, equal_layers);
  print_case("combined small details", combined_small_details, equal_layers);

  std::cout << "  traversability-heavy config\n";
  print_case("zero", zero_details, traversability_heavy);
  print_case("occupancy detail", occupancy_details, traversability_heavy);
  print_case("color/traversability detail", color_details,
             traversability_heavy);
  print_case("combined small details", combined_small_details,
             traversability_heavy);
}

void printStats(const char* label, const ErrorStats& stats,
                size_t sample_count) {
  std::cout << label << "\n";
  std::cout << "  occupancy error sum: " << stats.occupancy_sum << "\n";
  std::cout << "  occupancy mean abs error: "
            << stats.occupancy_sum / static_cast<float>(sample_count) << "\n";
  std::cout << "  max occupancy error: " << stats.max_occupancy << "\n";
  std::cout << "  rgb error sum: " << stats.rgb_sum << "\n";
  std::cout << "  rgb mean abs error: "
            << stats.rgb_sum / static_cast<float>(sample_count) << "\n";
  std::cout << "  max rgb error: " << stats.max_rgb << "\n";
  std::cout << "  traversability error sum: " << stats.traversability_sum
            << "\n";
  std::cout << "  traversability mean abs error: "
            << stats.traversability_sum / static_cast<float>(sample_count)
            << "\n";
  std::cout << "  max traversability error: " << stats.max_traversability
            << "\n";
}

PruningRunStats runPruningExperiment(
    const std::string& dataset, const std::string& label,
    const wavemap::HashedWaveletOctreeConfig& config,
    const ContinuousWaveletMap::PruningConfig& pruning_config,
    const std::vector<Sample>& samples) {
  PruningRunStats stats;
  stats.dataset = dataset;
  stats.label = label;

  ContinuousWaveletMap map(config, ContinuousWaveletMap::ThresholdConfig{}, pruning_config);

  const auto write_start = std::chrono::steady_clock::now();
  for (const auto& sample : samples) {
    map.setVoxelValue(sample.index, sample.expected);
  }
  const auto write_end = std::chrono::steady_clock::now();

  const auto read_before_start = std::chrono::steady_clock::now();
  const std::vector<LayeredVoxel> before_prune_values =
      readBackAll(map, samples);
  const auto read_before_end = std::chrono::steady_clock::now();

  stats.before_expected = computeErrorsAgainstExpected(map, samples);
  stats.nodes_before = map.size();
  stats.memory_before = map.getMemoryUsage();

  const auto prune_start = std::chrono::steady_clock::now();
  map.threshold();
  map.prune();
  const auto prune_end = std::chrono::steady_clock::now();

  stats.after_expected = computeErrorsAgainstExpected(map, samples);
  stats.prune_delta =
      computeErrorsAgainstBaseline(map, samples, before_prune_values);
  stats.nodes_after = map.size();
  stats.memory_after = map.getMemoryUsage();

  stats.write_us = std::chrono::duration_cast<std::chrono::microseconds>(
                       write_end - write_start)
                       .count();
  stats.read_before_us =
      std::chrono::duration_cast<std::chrono::microseconds>(read_before_end -
                                                            read_before_start)
          .count();
  stats.threshold_prune_us =
      std::chrono::duration_cast<std::chrono::microseconds>(prune_end -
                                                            prune_start)
          .count();
  return stats;
}

void printRunStats(const PruningRunStats& stats, size_t sample_count) {
  std::cout << "Pruning loss test [" << stats.dataset << "] (" << stats.label
            << ")\n";
  std::cout << "  samples: " << sample_count << "\n";
  std::cout << "  nodes before prune: " << stats.nodes_before << "\n";
  std::cout << "  nodes after prune: " << stats.nodes_after << "\n";
  std::cout << "  memory before prune bytes: " << stats.memory_before << "\n";
  std::cout << "  memory after prune bytes: " << stats.memory_after << "\n";
  std::cout << "  write time [us]: " << stats.write_us << "\n";
  std::cout << "  read-before-threshold time [us]: " << stats.read_before_us
            << "\n";
  std::cout << "  threshold+prune time [us]: " << stats.threshold_prune_us
            << "\n";

  printStats("Error vs original values before prune", stats.before_expected,
             sample_count);
  printStats("Error vs original values after prune", stats.after_expected,
             sample_count);
  printStats("Extra error introduced by threshold+prune", stats.prune_delta,
             sample_count);
}

int main() {
  printCoefficientPolicyCheck();
  std::cout << "\n";

  wavemap::HashedWaveletOctreeConfig config;
  config.min_cell_width = 0.1f;
  config.min_log_odds = -100.f;
  config.max_log_odds = 100.f;
  config.tree_height = 4;

  const std::vector<Sample> samples = makeLargeSamples();
  const std::vector<Sample> layer_boundary_samples =
      makeLayerBoundarySamples();
  const std::vector<std::pair<std::string, ContinuousWaveletMap::PruningConfig>> configs{
      {"occupancy-only", makeLayeredPruningConfig(1.f, 0.f, 0.f)},
      {"equal weighted layers", makeLayeredPruningConfig(1.f, 1.f, 1.f)},
      {"traversability-heavy", makeLayeredPruningConfig(1.f, 1.f, 2.f)}};
  const std::vector<std::pair<std::string, ContinuousWaveletMap::PruningConfig>>
      coarse_occupancy_configs{
          {"coarse occupancy-only",
           makeLayeredPruningConfigWithScales(1.f, 0.f, 0.f, 1.f, 0.1f, 0.1f)},
          {"coarse occupancy + equal layers",
           makeLayeredPruningConfigWithScales(1.f, 1.f, 1.f, 1.f, 0.1f, 0.1f)},
          {"coarse occupancy + traversability-heavy",
           makeLayeredPruningConfigWithScales(4.f, 1.f, 1.f, 1.f, 0.1f, 0.1f)}};

  for (const auto& [label, pruning_config] : configs) {
    const PruningRunStats stats =
        runPruningExperiment("smooth-gradient", label, config, pruning_config,
                             samples);
    printRunStats(stats, samples.size());
    std::cout << "\n";
  }

  for (const auto& [label, pruning_config] : configs) {
    const PruningRunStats stats = runPruningExperiment(
        "layer-boundary/default-scale", label, config, pruning_config,
        layer_boundary_samples);
    printRunStats(stats, layer_boundary_samples.size());
    std::cout << "\n";
  }

  for (const auto& [label, pruning_config] : coarse_occupancy_configs) {
    const PruningRunStats stats = runPruningExperiment(
        "layer-boundary/coarse-occupancy", label, config, pruning_config,
        layer_boundary_samples);
    printRunStats(stats, layer_boundary_samples.size());
    std::cout << "\n";
  }

  std::cout << "Interpretation: score = sum(weight * magnitude / scale). "
               "A node is kept when the combined score is above the combined "
               "threshold. Setting a layer weight to 0 ignores that layer; "
               "increasing a weight makes that layer contribute more to the "
               "combined pruning decision.\n";

  return 0;
}
