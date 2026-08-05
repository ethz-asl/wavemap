#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "../common/example_layered_map_config.h"
#include <wavemap/layered/layered_map_io.h>

namespace {
bool expectFailure(const std::string& name, bool load_result) {
  const bool failed = !load_result;
  std::cout << "  " << name << ": " << (failed ? "pass" : "FAIL") << "\n";
  return failed;
}

bool replaceBytes(const std::filesystem::path& path, const std::string& from,
                  const std::string& to) {
  if (from.size() != to.size()) {
    return false;
  }

  std::ifstream input(path, std::ios::binary);
  if (!input.is_open()) {
    return false;
  }
  std::string data((std::istreambuf_iterator<char>(input)),
                   std::istreambuf_iterator<char>());
  const size_t pos = data.find(from);
  if (pos == std::string::npos) {
    return false;
  }
  data.replace(pos, from.size(), to);

  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output.is_open()) {
    return false;
  }
  output.write(data.data(), static_cast<std::streamsize>(data.size()));
  return static_cast<bool>(output);
}

bool writeHeaderOnlyFile(const std::filesystem::path& path,
                         const std::string& magic, uint32_t version) {
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output.is_open()) {
    return false;
  }
  return wavemap::layered::io::detail::writeString(output, magic) &&
         wavemap::layered::io::detail::writePod(output, version);
}

void fillExampleMap(ExampleLayeredMap& map) {
  const wavemap::Index3D a(0, 0, 0);
  const wavemap::Index3D b(1, 0, 0);
  map.continuousMap().setVoxelValue(
      a, makeLayeredVoxel(10.f, rgb(1.f, 0.f, 0.f), 0.1f));
  map.continuousMap().setVoxelValue(
      b, makeLayeredVoxel(11.f, rgb(0.f, 1.f, 0.f), 0.2f));
  map.discreteLayers().semantic.setValue(a, 1);
  map.discreteLayers().semantic.setValue(b, 2);
  map.discreteLayers().changed.setValue(a, false);
  map.discreteLayers().changed.setValue(b, true);
}
}  // namespace

int main() {
  ExampleLayeredMapConfig config;
  config.continuous_map.min_cell_width = 0.1f;
  config.continuous_map.min_log_odds = -100.f;
  config.continuous_map.max_log_odds = 100.f;
  config.continuous_map.tree_height = 3;
  config.discrete_compression.block_height = 1;

  const std::filesystem::path output_dir = "/home/ci/data/maps";
  std::filesystem::create_directories(output_dir);

  bool ok = true;
  std::cout << "LayeredMap save/load failure experiment\n";

  {
    ExampleLayeredMap loaded(config);
    ok &= expectFailure(
        "missing file fails",
        ExampleLayeredMapIo::load(output_dir / "missing_file.lwvmp", loaded));
  }

  {
    const auto path = output_dir / "bad_magic.lwvmp";
    if (!writeHeaderOnlyFile(path, "BADMP", wavemap::layered::io::detail::kVersion)) {
      std::cerr << "Failed to write bad magic test file.\n";
      return 1;
    }
    ExampleLayeredMap loaded(config);
    ok &= expectFailure("wrong magic fails",
                        ExampleLayeredMapIo::load(path, loaded));
  }

  {
    const auto path = output_dir / "bad_version.lwvmp";
    if (!writeHeaderOnlyFile(path, wavemap::layered::io::detail::kMagic,
                             wavemap::layered::io::detail::kVersion + 1u)) {
      std::cerr << "Failed to write bad version test file.\n";
      return 1;
    }
    ExampleLayeredMap loaded(config);
    ok &= expectFailure("unsupported version fails",
                        ExampleLayeredMapIo::load(path, loaded));
  }

  ExampleLayeredMap map(config);
  fillExampleMap(map);

  {
    const auto path = output_dir / "bad_layer_name.lwvmp";
    if (!ExampleLayeredMapIo::save(path, map) ||
        !replaceBytes(path, "semantic", "semantix")) {
      std::cerr << "Failed to create bad layer name test file.\n";
      return 1;
    }
    ExampleLayeredMap loaded(config);
    ok &= expectFailure("wrong discrete layer name fails",
                        ExampleLayeredMapIo::load(path, loaded));
  }

  {
    const auto path = output_dir / "bad_layer_type.lwvmp";
    if (!ExampleLayeredMapIo::save(path, map) ||
        !replaceBytes(path, "bool", "b00l")) {
      std::cerr << "Failed to create bad layer type test file.\n";
      return 1;
    }
    ExampleLayeredMap loaded(config);
    ok &= expectFailure("wrong discrete layer type fails",
                        ExampleLayeredMapIo::load(path, loaded));
  }

  if (!ok) {
    std::cout << "LayeredMap failure checks did not all fail as expected.\n";
    return 1;
  }
  return 0;
}
