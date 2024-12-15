#include <memory>

#include <wavemap/core/common.h>
#include <wavemap/core/utils/edit/transform.h>
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

  // Define a transformation that flips the map upside down, for illustration
  Transformation3D T_AB;
  T_AB.getRotation() = Rotation3D{0.f, 1.f, 0.f, 0.f};

  // Transform the map
  auto thread_pool = std::make_shared<ThreadPool>();  // Optional
  map = edit::transform(*map, T_AB, thread_pool);

  // Save the map
  const std::filesystem::path output_map_path =
      home_dir / "your_map_transformed.wvmp";
  success &= io::mapToFile(*map, output_map_path);
  CHECK(success);
}
