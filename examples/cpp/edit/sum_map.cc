#include <memory>

#include <wavemap/core/common.h>
#include <wavemap/core/utils/edit/crop.h>
#include <wavemap/core/utils/edit/sum.h>
#include <wavemap/core/utils/edit/transform.h>
#include <wavemap/io/file_conversions.h>

int main(int, char**) {
  // Load the map
  const std::filesystem::path home_dir = CHECK_NOTNULL(getenv("HOME"));
  const std::filesystem::path input_map_path = home_dir / "your_map.wvmp";
  wavemap::MapBase::Ptr map_base;
  bool success = wavemap::io::fileToMap(input_map_path, map_base);
  CHECK(success);

  // Downcast it to a concrete map type
  auto map = std::dynamic_pointer_cast<wavemap::HashedWaveletOctree>(map_base);
  CHECK_NOTNULL(map);

  // Crop the map
  const wavemap::Point3D t_W_center{-2.2, -1.4, 0.0};
  const wavemap::FloatingPoint radius = 3.0;
  auto thread_pool = std::make_shared<wavemap::ThreadPool>();  // Optional
  wavemap::edit::crop_to_sphere(*map, t_W_center, radius, 0, thread_pool);

  // Create a translated copy
  wavemap::Transformation3D T_AB;
  T_AB.getPosition() = {5.0, 5.0, 0.0};
  auto map_translated = wavemap::edit::transform(*map, T_AB, thread_pool);

  // Merge them together
  wavemap::edit::sum(*map, *map_translated);

  // Save the map
  const std::filesystem::path output_map_path =
      home_dir / "your_map_merged.wvmp";
  success &= wavemap::io::mapToFile(*map, output_map_path);
  CHECK(success);
}
