#include <cstring>
#include <iostream>
#include <sstream>
#include <string>

#include <pcl/io/ply_io.h>
#include <wavemap_2d/utils/eigen_format.h>

#include "wavemap_2d_ground_truth/common.h"
#include "wavemap_2d_ground_truth/occupancy_grid_creator.h"

DEFINE_string(ply_input_filepath, "",
              "Path to the PLY file describing the geometry of the objects for "
              "which a ground truth occupancy grid should be created.");
DEFINE_string(
    occupancy_grid_output_filepath, "",
    "Path to a file where the ground truth occupancy grid should be stored.");
DEFINE_double(resolution, 0.0,
              "Resolution of the ground truth occupancy grid, expressed as the "
              "width of each cell in m.");
DEFINE_double(
    slice_height, 0.0,
    "Height at which the ground truth occupancy grid should be created, which "
    "will be a 2D slice of thickness equal to the resolution.");
DEFINE_double(scale_factor, 0.0,
              "Factor by which to scale the geometry in the PLY file.");
DEFINE_double(
    x, 0.0,
    "X component of the translation to apply to the geometry in the PLY file.");
DEFINE_double(
    y, 0.0,
    "Y component of the translation to apply to the geometry in the PLY file.");
DEFINE_double(
    z, 0.0,
    "Z component of the translation to apply to the geometry in the PLY file.");
DEFINE_double(Qx, 0.0,
              "Qx component of the quaternion of the rotation to apply to the "
              "geometry in the PLY file.");
DEFINE_double(Qy, 0.0,
              "Qy component of the quaternion of the rotation to apply to the "
              "geometry in the PLY file.");
DEFINE_double(Qz, 0.0,
              "Qz component of the quaternion of the rotation to apply to the "
              "geometry in the PLY file.");
DEFINE_double(Qw, 0.0,
              "Qw component of the quaternion of the rotation to apply to the "
              "geometry in the PLY file.");

using namespace wavemap_2d;  // NOLINT
int main(int argc, char* argv[]) {
  std::stringstream required_args;
  required_args << "ply_input_filepath occupancy_grid_output_filepath "
                   "resolution slice_height scale_factor x y z Qx Qy Qz Qw";

  /* Setup logging */
  google::SetUsageMessage(
      "This script can be used to create ground truth 2D occupancy grids from "
      "geometry specified in a ply file. To use it, please set the "
      "following args: " +
      required_args.str());
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::ParseCommandLineFlags(&argc, &argv, false);

  /* Process the command line arguments */
  std::string missing_flags;
  std::string flag_name;
  while (required_args >> flag_name) {
    if (google::GetCommandLineFlagInfoOrDie(flag_name.c_str()).is_default) {
      missing_flags += flag_name + " ";
    }
  }
  if (!missing_flags.empty()) {
    std::cout << "Missing required flags: " << missing_flags
              << "\nFor more information, call this program with --help.\n";
    return -1;
  }
  Transformation3D transform;
  transform.getPosition() =
      Transformation3D::Position(FLAGS_x, FLAGS_y, FLAGS_z);
  transform.getRotation().setValues(FLAGS_Qx, FLAGS_Qy, FLAGS_Qz, FLAGS_Qw);
  // NOTE: We don't check the quaternion's norm, since this is already done in
  //       Rotation::setValues(...)

  // Print the transformation params for debugging
  LOG(INFO) << "Will apply transformation:\n"
            << "- scaling: " << FLAGS_scale_factor << "\n"
            << "- translation: "
            << wavemap_2d::EigenFormat::oneLine(transform.getPosition()) << "\n"
            << "- rotation: "
            << wavemap_2d::EigenFormat::oneLine(
                   transform.getRotation().vector());

  /* Load the PLY file */
  pcl::PolygonMesh mesh;
  LOG(INFO) << "Importing .ply file: " << FLAGS_ply_input_filepath;
  pcl::io::loadPLYFile(FLAGS_ply_input_filepath, mesh);
  pcl::PointCloud<pcl::PointXYZ> vertex_coordinates;
  pcl::fromPCLPointCloud2(mesh.cloud, vertex_coordinates);

  // Initialize the occupancy grid creator
  LOG(INFO) << "Generating the occupancy grid";
  ground_truth::OccupancyGridCreator occupancy_grid_creator(FLAGS_resolution,
                                                            FLAGS_slice_height);

  // Iterate over all triangles
  size_t triangle_i = 0;
  size_t num_triangles = mesh.polygons.size();
  for (const pcl::Vertices& polygon : mesh.polygons) {
    // Ensure that the polygon is a triangle (other meshes are not supported)
    CHECK_EQ(polygon.vertices.size(), 3);

    // Indicate progress
    triangle_i++;
    // Only print progress for each promile of completion, to reduce IO wait
    if (triangle_i % (num_triangles / 1000) == 0) {
      wavemap_2d::Index occupancy_grid_dimensions =
          occupancy_grid_creator.getOccupancyGrid().dimensions();
      printf("\rProgress: %3.1f%% - map dimensions [%i, %i]",
             static_cast<wavemap_2d::FloatingPoint>(triangle_i) /
                 static_cast<wavemap_2d::FloatingPoint>(num_triangles) * 100,
             occupancy_grid_dimensions.x(), occupancy_grid_dimensions.y());
      std::cout << std::flush;
    }

    // Extract the triangle's vertices from the vertex coordinate pointcloud
    const Point3D vertex_a =
        vertex_coordinates[polygon.vertices[0]].getVector3fMap();
    const Point3D vertex_b =
        vertex_coordinates[polygon.vertices[1]].getVector3fMap();
    const Point3D vertex_c =
        vertex_coordinates[polygon.vertices[2]].getVector3fMap();

    // Transform the vertices from mesh frame into world frame
    wavemap_2d::ground_truth::Triangle triangle;
    triangle.vertex_a = transform * (FLAGS_scale_factor * vertex_a);
    triangle.vertex_b = transform * (FLAGS_scale_factor * vertex_b);
    triangle.vertex_c = transform * (FLAGS_scale_factor * vertex_c);

    // Update the occupancy grid with the new triangle
    occupancy_grid_creator.integrateTriangle(triangle);
  }
  LOG(INFO) << "Distance field building complete.";

  // Optionally floodfill unoccupied space.
  // TODO(victorr): Read from optional param
  constexpr bool floodfill_unoccupied = false;
  if (floodfill_unoccupied) {
    occupancy_grid_creator.floodfillUnoccupied(4 * FLAGS_resolution);
  }

  /* Check if the map is empty before continuing */
  if (occupancy_grid_creator.getOccupancyGrid().empty()) {
    LOG(INFO) << "No visuals to publish. The map is empty.";
    return -1;
  }

  /* Publish debugging visuals */
  // TODO(victorr): Read from optional param
  constexpr bool publish_debug_visuals = true;
  if (publish_debug_visuals) {
    LOG(INFO) << "Publishing visuals";
    occupancy_grid_creator.getOccupancyGrid().showImage(true, 3000);
  }

  // Save the occupancy grid to a file
  LOG(INFO) << "Saving occupancy grid to file: "
            << FLAGS_occupancy_grid_output_filepath;
  occupancy_grid_creator.getOccupancyGrid().save(
      FLAGS_occupancy_grid_output_filepath,
      ground_truth::kSaveWithFloatingPrecision);

  LOG(INFO) << "Done";

  return 0;
}
