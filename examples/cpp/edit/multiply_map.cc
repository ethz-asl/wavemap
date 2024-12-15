#include <memory>

#include <wavemap/core/common.h>
#include <wavemap/core/utils/edit/multiply.h>
#include <wavemap/io/file_conversions.h>

using namespace wavemap;  // NOLINT
int main(int, char**) {
  // Load the map
  const std::filesystem::path home_dir = CHECK_NOTNULL(getenv("HOME"));
  const std::filesystem::path input_map_path = home_dir / "your_map.wvmp";
  MapBase::Ptr map_base;
  bool success = io::fileToMap(input_map_path, map_base);
  CHECK(success);

  // Downcast it to a concrete map type
  auto map = std::dynamic_pointer_cast<HashedWaveletOctree>(map_base);
  CHECK_NOTNULL(map);

  // Use the multiply method to implement exponential forgetting
  const FloatingPoint decay_factor = 0.9;
  auto thread_pool = std::make_shared<ThreadPool>();  // Optional
  edit::multiply(*map, decay_factor, thread_pool);

  // Save the map
  const std::filesystem::path output_map_path =
      home_dir / "your_map_decayed.wvmp";
  success &= io::mapToFile(*map, output_map_path);
  CHECK(success);
}
