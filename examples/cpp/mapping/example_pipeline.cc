#include <iostream>
#include <string>

#include <wavemap/core/map/map_factory.h>
#include <wavemap/io/config/file_conversions.h>
#include <wavemap/io/map/file_conversions.h>
#include <wavemap/pipeline/pipeline.h>

using namespace wavemap;
int main(int, char** argv) {
  // Settings
  const std::string config_name = "example_config.yaml";
  const std::string output_map_name = "example_map.wvmp";
  const std::filesystem::path current_dir =
      std::filesystem::canonical(__FILE__).parent_path();
  const std::filesystem::path config_path = current_dir / config_name;
  const std::filesystem::path output_map_path = current_dir / output_map_name;

  // Load the config
  std::cout << "Loading config file: " << config_path << std::endl;
  const auto params = io::yamlFileToParams(config_path);
  CHECK(params.has_value());

  // Create the map
  const auto map_config = params->getChild("map");
  CHECK(map_config.has_value());
  MapBase::Ptr map = MapFactory::create(map_config.value());
  CHECK_NOTNULL(map);

  // Create measurement integration pipeline
  Pipeline pipeline{map};

  // Add map operations to pipeline
  const auto map_operations =
      params->getChildAs<param::Array>("map_operations");
  CHECK(map_operations);
  for (const auto& operation_params : map_operations.value()) {
    pipeline.addOperation(operation_params);
  }

  // Add measurement integrators to pipeline
  const auto measurement_integrators =
      params->getChildAs<param::Map>("measurement_integrators");
  CHECK(measurement_integrators);
  for (const auto& [integrator_name, integrator_params] :
       measurement_integrators.value()) {
    pipeline.addIntegrator(integrator_name, integrator_params);
  }

  // Define a depth camera image for illustration
  const int width = 640;
  const int height = 480;
  Image<> depth_image{width, height};
  depth_image.setToConstant(2.f);

  // Define a depth camera pose for illustration
  Transformation3D T_W_C{};
  // Set the camera's [x, y, z] position
  T_W_C.getPosition() = {0.f, 0.f, 0.f};
  // Set the camera's orientation
  // For example as a quaternion's [w, x, y, z] coefficients
  T_W_C.getRotation() = Rotation3D{0.5f, -0.5f, -0.5f, 0.5f};
  // NOTE: Alternatively, the rotation can also be loaded from a rotation
  //       matrix, or T_W_C can be initialized from a transformation matrix.

  // Integrate a depth image
  pipeline.runPipeline({"your_camera"}, PosedImage<>{T_W_C, depth_image});

  // See if the map was updated
  const size_t map_size_KB = map->getMemoryUsage() / 1024;
  std::cout << "Created map of size: " << map_size_KB << " KB" << std::endl;

  // Save map
  std::cout << "Saving it to: " << output_map_path << std::endl;
  io::mapToFile(*map, output_map_path);
}
