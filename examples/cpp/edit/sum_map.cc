#include <memory>

#include <wavemap/core/common.h>
#include <wavemap/core/utils/edit/crop.h>
#include <wavemap/core/utils/edit/sum.h>
#include <wavemap/core/utils/edit/transform.h>
#include <wavemap/core/utils/geometry/aabb.h>
#include <wavemap/core/utils/geometry/sphere.h>
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

  // Set a box in the map to free
  AABB<Point3D> box{{6.f, 6.f, -2.f}, {10.f, 10.f, 2.f}};
  edit::sum(*map, box, -1.f, thread_pool);

  // Set a sphere in the map to occupied
  Sphere<Point3D> sphere{{8.f, 8.f, 0.f}, 1.5f};
  edit::sum(*map, sphere, 2.f, thread_pool);

  // Save the map
  const std::filesystem::path output_map_path =
      home_dir / "your_map_merged.wvmp";
  success &= io::mapToFile(*map, output_map_path);
  CHECK(success);
}
