#include <memory>

#include <wavemap/core/common.h>
#include <wavemap/core/utils/edit/crop.h>
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

  // Crop the map
  const Point3D t_W_center{-2.2, -1.4, 0.0};
  const FloatingPoint radius = 3.0;
  const Sphere cropping_sphere{t_W_center, radius};
  auto thread_pool = std::make_shared<ThreadPool>();  // Optional
  edit::crop(*map, cropping_sphere, 0, thread_pool);

  // Save the map
  const std::filesystem::path output_map_path =
      home_dir / "your_map_cropped.wvmp";
  success &= io::mapToFile(*map, output_map_path);
  CHECK(success);
}
