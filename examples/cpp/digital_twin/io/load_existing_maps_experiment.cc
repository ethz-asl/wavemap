#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

#include <wavemap/core/map/hashed_blocks.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/core/map/volumetric_octree.h>
#include <wavemap/core/map/wavelet_octree.h>
#include <wavemap/io/file_conversions.h>

#include "../common/example_layered_map_config.h"

namespace {
void printUsage(const char* executable_name) {
  std::cerr << "Usage: " << executable_name
            << " <map.wvmp> <layered_map.lwvmp>\n";
}

void printLayeredMapSchema(const wavemap::layered::LayeredMapSchema& schema) {
  std::cout << "  schema continuous layers:";
  if (schema.continuous_layers.empty()) {
    std::cout << " none";
  }
  std::cout << "\n";
  for (const auto& layer : schema.continuous_layers) {
    std::cout << "    - " << layer.name << " (" << layer.type << ")\n";
  }

  std::cout << "  schema discrete layers:";
  if (schema.discrete_layers.empty()) {
    std::cout << " none";
  }
  std::cout << "\n";
  for (const auto& layer : schema.discrete_layers) {
    std::cout << "    - " << layer.name << " (" << layer.type << ")\n";
  }
}

bool inspectWavemapFile(const std::filesystem::path& path) {
  wavemap::MapBase::Ptr map;
  if (!wavemap::io::fileToMap(path, map) || !map) {
    std::cerr << "Failed to load Wavemap file: " << path << "\n";
    return false;
  }

  std::cout << "Loaded Wavemap file: " << path << "\n";
  std::cout << "  min cell width: " << map->getMinCellWidth() << "\n";
  std::cout << "  tree height: " << map->getTreeHeight() << "\n";
  std::cout << "  log odds range: [" << map->getMinLogOdds() << ", "
            << map->getMaxLogOdds() << "]\n";

  if (const auto hashed_wavelet =
          std::dynamic_pointer_cast<wavemap::HashedWaveletOctree>(map)) {
    std::cout << "  type: HashedWaveletOctree\n";
    std::cout << "  allocated blocks: " << hashed_wavelet->getHashMap().size()
              << "\n";
  } else if (std::dynamic_pointer_cast<wavemap::WaveletOctree>(map)) {
    std::cout << "  type: WaveletOctree\n";
  } else if (const auto hashed_blocks =
                 std::dynamic_pointer_cast<wavemap::HashedBlocks>(map)) {
    std::cout << "  type: HashedBlocks\n";
    std::cout << "  allocated blocks: " << hashed_blocks->getHashMap().size()
              << "\n";
  } else if (std::dynamic_pointer_cast<wavemap::VolumetricOctree>(map)) {
    std::cout << "  type: VolumetricOctree\n";
  } else {
    std::cout << "  type: unknown MapBase subclass\n";
  }
  return true;
}

bool inspectLayeredMapFile(const std::filesystem::path& path) {
  ExampleLayeredMapConfig config;
  ExampleLayeredMap map(config);
  const wavemap::layered::LayeredMapSchema expected_schema =
      exampleLayeredMapSchema(map);

  wavemap::layered::LayeredMapSchema file_schema;
  if (wavemap::layered::io::readLayeredMapSchema(path, file_schema)) {
    std::cout << "Read LayeredMap schema before full load: " << path << "\n";
    printLayeredMapSchema(file_schema);

    std::string compatibility_error;
    if (wavemap::layered::checkLayeredMapSchemaCompatibility(
            file_schema, expected_schema, &compatibility_error)) {
      std::cout << "  schema compatibility: pass\n";
    } else {
      std::cout << "  schema compatibility: FAIL\n";
      std::cout << "  " << compatibility_error << "\n";
    }
  } else {
    std::cout << "Could not read LayeredMap schema before full load: " << path
              << "\n";
    std::cout << "  Note: this can happen for invalid/non-layered files, or "
                 ".lwvmp files written before schema metadata was added.\n";
  }

  std::string load_error;
  if (!ExampleLayeredMapIo::load(path, map, &load_error)) {
    std::cerr << "Failed to load LayeredMap file: " << path << "\n";
    if (!load_error.empty()) {
      std::cerr << "  " << load_error << "\n";
    }
    std::cerr << "  Note: .lwvmp files are typed. This inspector expects the "
                 "ExampleLayeredMap schema.\n";
    return false;
  }

  std::cout << "Loaded LayeredMap file: " << path << "\n";
  std::cout << "  continuous min cell width: "
            << map.continuousMap().getMinCellWidth() << "\n";
  std::cout << "  continuous tree height: " << map.continuousMap().getTreeHeight()
            << "\n";
  std::cout << "  continuous allocated blocks: "
            << map.continuousMap().getHashMap().size() << "\n";
  std::cout << "  semantic parent cells: "
            << map.discreteLayers().semantic.parentCount() << "\n";
  std::cout << "  semantic exceptions: "
            << map.discreteLayers().semantic.exceptionCount() << "\n";
  std::cout << "  changed parent cells: "
            << map.discreteLayers().changed.parentCount() << "\n";
  std::cout << "  changed exceptions: "
            << map.discreteLayers().changed.exceptionCount() << "\n";
  return true;
}
}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    printUsage(argv[0]);
    return 1;
  }

  const bool wavemap_ok = inspectWavemapFile(argv[1]);
  const bool layered_ok = inspectLayeredMapFile(argv[2]);
  return wavemap_ok && layered_ok ? 0 : 1;
}
