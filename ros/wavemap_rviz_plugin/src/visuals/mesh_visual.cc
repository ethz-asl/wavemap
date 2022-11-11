#include "wavemap_rviz_plugin/visuals/mesh_visual.h"

#include <unordered_map>

#include <wavemap_common/indexing/index_conversions.h>
#include <wavemap_common/indexing/index_hashes.h>
#include <wavemap_common/iterator/grid_iterator.h>

#include "wavemap_rviz_plugin/marching_cubes.h"

namespace wavemap::rviz_plugin {
MeshVisual::MeshVisual(Ogre::SceneManager* scene_manager,
                       Ogre::SceneNode* parent_node)
    : frame_node_(parent_node->createChildSceneNode()),
      scene_manager_(scene_manager),
      mesh_object_(
          scene_manager_->createManualObject("wavemap_isosurface_mesh")) {
  frame_node_->attachObject(mesh_object_);
}

MeshVisual::~MeshVisual() {
  // Destroy the frame node
  frame_node_->detachObject(mesh_object_);
  scene_manager_->destroyManualObject(mesh_object_);
  scene_manager_->destroySceneNode(frame_node_);
}

void MeshVisual::loadMap(const VolumetricDataStructure3D& map,
                         FloatingPoint min_occupancy_log_odds,
                         FloatingPoint max_occupancy_log_odds,
                         FloatingPoint alpha) {
  // Constants
  const FloatingPoint min_cell_width = map.getMinCellWidth();
  const FloatingPoint min_value_threshold =
      logOddsToValue(min_occupancy_log_odds);
  const FloatingPoint max_value_threshold =
      logOddsToValue(max_occupancy_log_odds);

  // Collect all the voxels that contain surface crossings
  std::unordered_map<Index3D, std::array<FloatingPoint, 8>, VoxbloxIndexHash<3>>
      surface_voxels;
  map.forEachLeaf([=, &map, &surface_voxels](const OctreeIndex& node_index,
                                             FloatingPoint cell_log_odds) {
    if (cell_log_odds < max_occupancy_log_odds) {
      return;
    }

    for (const Index3D& cell_index :
         Grid<3>(convert::nodeIndexToMinCornerIndex(node_index),
                 convert::nodeIndexToMaxCornerIndex(node_index))) {
      // Get the neighboring cell values
      std::array<FloatingPoint, 27> neighbor_values{};
      for (LinearIndex neighbor_offset = 0; neighbor_offset < 27;
           ++neighbor_offset) {
        const Index3D neighbor_index =
            cell_index + convert::linearIndexToIndex<3, 3>(neighbor_offset) -
            Index3D::Ones();
        const FloatingPoint neighbor_log_odds =
            map.getCellValue(neighbor_index);
        neighbor_values[neighbor_offset] = logOddsToValue(neighbor_log_odds);
      }

      std::array<FloatingPoint, 8> voxel_corner_values{};
      for (LinearIndex min_corner_offset = 0; min_corner_offset < 8;
           ++min_corner_offset) {
        for (LinearIndex corner_idx = 0; corner_idx < 8; ++corner_idx) {
          const Index3D corner_offset =
              MarchingCubes::kCornerOffsets[corner_idx];
          const LinearIndex voxel_offset = convert::indexToLinearIndex<3, 3>(
              convert::linearIndexToIndex<2, 3>(min_corner_offset) +
              corner_offset);
          voxel_corner_values[corner_idx] = neighbor_values[voxel_offset];
        }
        const auto [min_value, max_value] = std::minmax_element(
            voxel_corner_values.begin(), voxel_corner_values.end());
        if (*min_value < min_value_threshold &&
            max_value_threshold < *max_value) {
          const Index3D voxel_min_corner =
              cell_index +
              convert::linearIndexToIndex<2, 3>(min_corner_offset) -
              Index3D::Ones();
          surface_voxels[voxel_min_corner] = voxel_corner_values;
        }
      }
    }
  });

  // Convert the surface voxels to triangles using Marching Cubes
  std::vector<Point3D> vertices;
  for (const auto& voxel : surface_voxels) {
    const Point3D min_corner =
        convert::indexToMinCorner(voxel.first, min_cell_width);
    Eigen::Matrix<FloatingPoint, 3, 8> corner_positions;
    for (int corner_idx = 0; corner_idx < 8; ++corner_idx) {
      const Vector3D corner_offset =
          min_cell_width *
          MarchingCubes::kCornerOffsets[corner_idx].cast<FloatingPoint>();
      corner_positions.col(corner_idx) = min_corner + corner_offset;
    }
    MarchingCubes::meshCube(corner_positions, voxel.second, vertices);
  }

  // Weld near-identical vertices
  std::vector<LinearIndex> indices(vertices.size());
  {
    constexpr FloatingPoint kTolerance = 1e-4f;
    for (auto& vertex : vertices) {
      vertex = (vertex / kTolerance).array().round() * kTolerance;
    }
    const auto vertices_original = vertices;
    std::sort(vertices.begin(), vertices.end(), lessThan);
    vertices.erase(std::unique(vertices.begin(), vertices.end()),
                   vertices.end());
    for (const Point3D& vertex_original : vertices_original) {
      const auto lower_bound = std::lower_bound(
          vertices.begin(), vertices.end(), vertex_original, lessThan);
      const LinearIndex index = std::distance(vertices.begin(), lower_bound);
      CHECK_GE(index, 0);
      CHECK_LT(index, vertices.size());
      indices.emplace_back(index);
    }
  }

  // Compute normals
  CHECK(indices.size() % 3 == 0);
  std::vector<Vector3D> normals(vertices.size(), Vector3D::Zero());
  const size_t num_triangles = indices.size() / 3;
  {
    for (LinearIndex triangle_idx = 0; triangle_idx < num_triangles;
         ++triangle_idx) {
      const LinearIndex index_0 = indices[3 * triangle_idx];
      const LinearIndex index_1 = indices[3 * triangle_idx + 1];
      const LinearIndex index_2 = indices[3 * triangle_idx + 2];
      const Point3D vertex_0 = vertices[index_0];
      const Point3D vertex_1 = vertices[index_1];
      const Point3D vertex_2 = vertices[index_2];
      const Vector3D tx = (vertex_1 - vertex_0);
      const Vector3D ty = (vertex_2 - vertex_0);
      const Vector3D normal = tx.cross(ty).normalized();
      normals[index_0] += normal;
      normals[index_1] += normal;
      normals[index_2] += normal;
    }
    for (auto& normal : normals) {
      normal.normalize();
    }
  }

  // Clear the previous mesh and repopulate it with the new triangles
  // TODO(victorr): Switch to using beginUpdate() to update the mesh
  mesh_object_->clear();
  mesh_object_->begin("BaseWhiteNoLighting",
                      Ogre::RenderOperation::OT_TRIANGLE_LIST);
  CHECK_EQ(vertices.size(), normals.size());
  for (LinearIndex index = 0u; index < vertices.size(); ++index) {
    const Point3D vertex = vertices[index];
    mesh_object_->position(vertex.x(), vertex.y(), vertex.z());

    const Vector3D normal = normals[index];
    mesh_object_->normal(normal.x(), normal.y(), normal.z());

    Ogre::ColourValue color;
    color.r = 0.5f * normal.x() + 0.5f;
    color.g = 0.5f * normal.y() + 0.5f;
    color.b = 0.5f * normal.z() + 0.5f;
    color.a = alpha;
    mesh_object_->colour(color);
  }
  for (const LinearIndex& index : indices) {
    CHECK_GE(index, 0u);
    CHECK_LT(index, vertices.size());
    mesh_object_->index(index);
  }
  mesh_object_->end();
}

// Position and orientation are passed through to the SceneNode
void MeshVisual::setFramePosition(const Ogre::Vector3& position) {
  frame_node_->setPosition(position);
}

void MeshVisual::setFrameOrientation(const Ogre::Quaternion& orientation) {
  frame_node_->setOrientation(orientation);
}
}  // namespace wavemap::rviz_plugin
