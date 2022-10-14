#include "wavemap_ground_truth/user_interfaces/gazebo_plugin.h"

#include <string>

#include "wavemap_ground_truth/common.h"
#include "wavemap_ground_truth/occupancy_grid_creator.h"

namespace gazebo {
GZ_REGISTER_WORLD_PLUGIN(Wavemap2DGroundTruthPlugin)

Wavemap2DGroundTruthPlugin::Wavemap2DGroundTruthPlugin()
    : WorldPlugin(), nh_private_("~/wavemap_ground_truth_plugin") {}

void Wavemap2DGroundTruthPlugin::Load(physics::WorldPtr world,
                                      sdf::ElementPtr _sdf) {
  world_ = world;

  // Advertise the TSDF generation service
  std::string service_name = "save_occupancy_grid_to_file";
  srv_ = nh_private_.advertiseService(
      service_name, &Wavemap2DGroundTruthPlugin::serviceCallback, this);
  LOG(INFO) << "Advertising service: " << srv_.getService();
}

bool Wavemap2DGroundTruthPlugin::serviceCallback(
    wavemap_msgs::FilePath::Request& request,
    wavemap_msgs::FilePath::Response& response) {
  response.success = saveOccupancyGrid(request.file_path);
  return true;
}

bool Wavemap2DGroundTruthPlugin::saveOccupancyGrid(
    const std::string& file_path) {
  constexpr bool kDebug = true;

  // Read the resolution and slice height from ROS params
  wavemap::FloatingPoint resolution;
  wavemap::FloatingPoint slice_height;
  if (!nh_private_.getParam("resolution", resolution)) {
    LOG(WARNING) << "ROS param " << nh_private_.resolveName("resolution")
                 << " must be set.";
    return false;
  }
  if (!nh_private_.getParam("slice_height", slice_height)) {
    LOG(WARNING) << "ROS param " << nh_private_.resolveName("slice_height")
                 << " must be set.";
    return false;
  }

  // Instantiate the ground truth SDF creator
  wavemap::VolumetricDataStructureConfig data_structure_config;
  data_structure_config.min_cell_width = resolution;
  wavemap::ground_truth::OccupancyGridCreator occupancy_grid_creator(
      data_structure_config, slice_height);

  // Instantiate a Gazebo mesh manager
  common::MeshManager* const mesh_manager = common::MeshManager::Instance();
  if (!mesh_manager) {
    LOG(ERROR) << "Could not get pointer to MeshManager";
    return false;
  }

  // Iterate over all collision geometries
#if GAZEBO_MAJOR_VERSION > 8
  for (const physics::ModelPtr& model : world_->Models()) {
#else
  for (const physics::ModelPtr& model : world_->GetModels()) {
#endif
    for (const physics::LinkPtr& link : model->GetLinks()) {
      for (const physics::CollisionPtr& collision : link->GetCollisions()) {
        LOG_IF(INFO, kDebug)
            << "Processing '" << collision->GetScopedName(true) << "'";

        // Convert the geometry shape to a proto message
        // NOTE: This is done such that we can read the shape properties through
        //       methods from msgs::Geometry, whose names are human friendlier
        msgs::Geometry geometry_msg;
        collision->GetShape()->FillMsg(geometry_msg);

        LOG_IF(INFO, kDebug) << "------------------ SDF ------------------";
        LOG_IF(INFO, kDebug) << collision->GetShape()->GetSDF()->ToString("");
        LOG_IF(INFO, kDebug) << "-----------------------------------------";

        if (!geometry_msg.has_type()) {
          LOG(ERROR) << "Geometry type not available";
          return false;
        }

        std::string geometry_type_str =
            msgs::ConvertGeometryType(geometry_msg.type());

        // TODO(victorr): Add support for remaining Mesh shapes, namely
        //                - physics::Base::POLYLINE_SHAPE
        //                - physics::Base::HEIGHTMAP_SHAPE
        //                - physics::Base::MAP_SHAPE
        //                - physics::Base::MULTIRAY_SHAPE
        //                - physics::Base::RAY_SHAPE
        if (!(geometry_type_str == "box" || geometry_type_str == "cylinder" ||
              geometry_type_str == "sphere" || geometry_type_str == "plane" ||
              geometry_type_str == "mesh")) {
          LOG(ERROR) << "Not yet able to process shapes of type: "
                     << geometry_type_str;
          return false;
        }

        const common::Mesh* mesh_ptr = nullptr;
        std::string mesh_name;
        if (geometry_type_str == "mesh") {
          // find base name of mesh object
          std::string mesh_base_name = geometry_msg.mesh().filename();
          LOG_IF(INFO, kDebug) << "Attempting to load mesh " << mesh_base_name;
          // extracting file name
          size_t idx = mesh_base_name.find('.');
          mesh_base_name.erase(mesh_base_name.begin() + idx,
                               mesh_base_name.end());
          const std::string prefix = "file://";
          size_t idx_prefix = mesh_base_name.find(prefix);
          if (idx_prefix < mesh_base_name.size() &&
              mesh_base_name.size() > prefix.size()) {
            mesh_base_name.erase(mesh_base_name.begin(),
                                 mesh_base_name.begin() + prefix.size());
          }
          // try loading different mesh objects
          for (const std::string& object_type : kMeshFileExtensions) {
            mesh_name = mesh_base_name + object_type;
            mesh_ptr = mesh_manager->Load(mesh_name);
            if (mesh_ptr) {
              LOG_IF(INFO, kDebug)
                  << "- Loading file \"" << mesh_name << "\" successful.";
              break;
            } else {
              LOG_IF(INFO, kDebug)
                  << "- Loading mesh \"" << mesh_name << "\" failed.";
            }
          }
          if (!mesh_ptr) {
            mesh_name = geometry_msg.mesh().filename();
            LOG(WARNING) << "All attempts to load mesh " << mesh_name
                         << " failed.";
          }
        } else {
          mesh_name = "unit_" + geometry_type_str;
          mesh_ptr = mesh_manager->GetMesh(mesh_name);
        }

        if (!mesh_ptr) {
          LOG(ERROR) << "Could not get pointer to mesh '" << mesh_name << "'";
          return false;
        }

        // iterate over sub meshes
        const size_t num_submeshes = mesh_ptr->GetSubMeshCount();
        for (size_t submesh_id = 0; submesh_id < num_submeshes; submesh_id++) {
          // Create a copy of the submesh s.t. it can be manipulated
          const common::SubMesh* submesh_original =
              mesh_ptr->GetSubMesh(submesh_id);
          if (!submesh_original) {
            LOG(ERROR) << "Could not get pointer to submesh nr" << submesh_id
                       << " of mesh named " << mesh_name;
            return false;
          }
          common::SubMesh submesh(submesh_original);

          // Make sure we're dealing with a triangle mesh
          // TODO(victorr): Add support for other mesh types, e.g.
          //                - common::SubMesh::LINES
          if (submesh.GetPrimitiveType() != common::SubMesh::TRIANGLES) {
            LOG(WARNING) << "Encountered a mesh with type "
                         << getMeshTypeStr(submesh.GetPrimitiveType())
                         << ". Currently, "
                         << "only triangular meshes are supported."
                         << "\nSkipping this mesh.";
            continue;
          }

          // Find the geometry size
          // NOTE: There is no need to scale the geometry, since
          //       Gazebo already returns it at the appropriate scale
          ignition::math::Vector3d geometry_size;
          if (geometry_type_str == "box") {
            geometry_size = msgs::ConvertIgn(geometry_msg.box().size());
          } else if (geometry_type_str == "sphere") {
            const double radius = geometry_msg.sphere().radius();
            geometry_size.Set(2.0 * radius, 2.0 * radius, 2.0 * radius);
          } else if (geometry_type_str == "cylinder") {
            const double radius = geometry_msg.cylinder().radius();
            const double length = geometry_msg.cylinder().length();
            geometry_size.Set(2.0 * radius, 2.0 * radius, length);
          } else if (geometry_type_str == "plane") {
            const msgs::Vector2d dimensions = geometry_msg.plane().size();
            geometry_size.Set(dimensions.x(), dimensions.y(), 1.0);
          } else if (geometry_type_str == "mesh") {
            // NOTE: The shape scale is absolute w.r.t. the world
            if (collision->GetShape()->GetSDF()->HasElement("scale")) {
              collision->GetShape()
                  ->GetSDF()
                  ->GetElement("scale")
                  ->GetValue()
                  ->Get(geometry_size);
            } else {
              geometry_size = collision->GetShape()->Scale();
            }
            LOG_IF(INFO, kDebug) << "Scale: shape_scale " << geometry_size;
          } else {
            LOG(ERROR) << "Could not get geometry size of "
                       << geometry_type_str;
            return false;
          }

          // Scale the mesh and transform it into world frame
#if GAZEBO_MAJOR_VERSION > 8
          const ignition::math::Pose3d transform = collision->WorldPose();
#else
          const ignition::math::Pose3d transform =
              collision->GetWorldPose().Ign();
#endif
          for (unsigned int vertex_i = 0; vertex_i < submesh.GetVertexCount();
               vertex_i++) {
            // Create a copy of the vertex s.t. it can be manipulated
            ignition::math::Vector3d new_vertex = submesh.Vertex(vertex_i);

            // Scale and transform it into world frame
            new_vertex *= geometry_size;
            new_vertex = transform.Rot() * new_vertex;
            new_vertex += transform.Pos();

            // Add the vertex to the mesh
            submesh.SetVertex(vertex_i, new_vertex);
          }

          // Integrate the mesh faces (triangles) into the SDF
          const unsigned int num_faces = submesh.GetIndexCount() / 3;
          LOG_IF(INFO, kDebug) << "Integrating " << num_faces << " faces";
          for (unsigned int triangle_i = 0; triangle_i < num_faces;
               triangle_i++) {
            // Get the indices of the vertices
            const unsigned int index_a = submesh.GetIndex(triangle_i * 3);
            const unsigned int index_b = submesh.GetIndex(triangle_i * 3 + 1);
            const unsigned int index_c = submesh.GetIndex(triangle_i * 3 + 2);

            // Get the coordinates of the vertices
            wavemap::ground_truth::Triangle triangle;
            triangle.vertex_a = {
                static_cast<float>(submesh.Vertex(index_a).X()),
                static_cast<float>(submesh.Vertex(index_a).Y()),
                static_cast<float>(submesh.Vertex(index_a).Z())};
            triangle.vertex_b = {
                static_cast<float>(submesh.Vertex(index_b).X()),
                static_cast<float>(submesh.Vertex(index_b).Y()),
                static_cast<float>(submesh.Vertex(index_b).Z())};
            triangle.vertex_c = {
                static_cast<float>(submesh.Vertex(index_c).X()),
                static_cast<float>(submesh.Vertex(index_c).Y()),
                static_cast<float>(submesh.Vertex(index_c).Z())};

            // Integrate the triangle into the mesh
            occupancy_grid_creator.integrateTriangle(triangle);
          }
        }
      }
    }
  }

  // Optionally floodfill unoccupied space.
  bool floodfill_unoccupied = false;
  nh_private_.param("floodfill_unoccupied", floodfill_unoccupied,
                    floodfill_unoccupied);
  if (floodfill_unoccupied) {
    LOG_IF(INFO, kDebug) << "Floodfill unoccupied space";
    occupancy_grid_creator.floodfillUnoccupied(wavemap::Index2D::Zero());
  }

  // Visualize the TSDF and intersection count layers
  bool publish_debug_visuals = true;
  nh_private_.param("publish_visuals", publish_debug_visuals,
                    publish_debug_visuals);
  if (publish_debug_visuals) {
    LOG_IF(INFO, kDebug) << "Publishing visuals";
    occupancy_grid_creator.getOccupancyGrid().showImage(true, 3000);
  }

  // Save the occupancy grid to a file
  LOG_IF(INFO, kDebug) << "Saving occupancy grid to file: " << file_path;
  occupancy_grid_creator.getOccupancyGrid().save(
      file_path, wavemap::ground_truth::kSaveWithFloatingPrecision);

  return true;
}
}  // namespace gazebo
