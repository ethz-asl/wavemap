#include <memory>

#include <wavemap/core/common.h>
#include <wavemap/core/utils/edit/crop.h>
#include <wavemap/core/utils/edit/sum.h>
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

  // Crop the map
  const Point3D t_W_center{-2.2, -1.4, 0.0};
  const FloatingPoint radius = 3.0;
  auto thread_pool = std::make_shared<ThreadPool>();  // Optional
  edit::crop_to_sphere(*map, t_W_center, radius, 0, thread_pool);

  // Create a translated copy
  Transformation3D T_AB;
  T_AB.getPosition() = {5.0, 5.0, 0.0};
  auto map_translated = edit::transform(*map, T_AB, thread_pool);

  // Merge them together
  edit::sum(*map, *map_translated);

  // Save the map
  const std::filesystem::path output_map_path =
      home_dir / "your_map_merged.wvmp";
  success &= io::mapToFile(*map, output_map_path);
  CHECK(success);
}
