#include "wavemap_rviz_plugin/visuals/mesh_visual.h"

#include <wavemap_common/indexing/index_conversions.h>
#include <wavemap_common/indexing/index_hashes.h>

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
  const int max_height = 14;
  const FloatingPoint min_threshold = min_occupancy_log_odds;
  const FloatingPoint max_threshold = max_occupancy_log_odds;

  // Clear the previous mesh before building the new one
  // TODO(victorr): Switch to using beginUpdate() to update the mesh
  mesh_object_->clear();
  mesh_object_->begin("BaseWhiteNoLighting",
                      Ogre::RenderOperation::OT_TRIANGLE_LIST);

  auto log_odds_to_prob = [](FloatingPoint log_odds) {
    const FloatingPoint odds = std::exp(log_odds);
    return odds / (1.f + odds);
  };
  std::unordered_map<Index3D, Ogre::uint32, VoxbloxIndexHash<3>> vertex_map;
  std::vector<Vector3D> zero_crossings;
  zero_crossings.reserve(12);
  map.forEachLeaf([=, &map, &vertex_map, &zero_crossings](
                      const OctreeIndex& cell_index,
                      FloatingPoint cell_log_odds) {
    if (cell_index.height != 0) {
      return;
    }

    // Get the neighboring cell values
    std::array<FloatingPoint, 8> neighbor_values{};
    OctreeIndex neighbor_node_index = cell_index;
    for (auto dx : {0, 1}) {
      neighbor_node_index.position.x() = cell_index.position.x() + dx;
      for (auto dy : {0, 1}) {
        neighbor_node_index.position.y() = cell_index.position.y() + dy;
        for (auto dz : {0, 1}) {
          neighbor_node_index.position.z() = cell_index.position.z() + dz;
          const Index3D neighbor_index =
              convert::nodeIndexToMinCornerIndex(neighbor_node_index);
          const FloatingPoint neighbor_log_odds =
              map.getCellValue(neighbor_index);
          neighbor_values[convert::indexToLinearIndex<2, 3>({dx, dy, dz})] =
              log_odds_to_prob(neighbor_log_odds) - 0.5f;
        }
      }
    }
    const auto [min_value, max_value] =
        std::minmax_element(neighbor_values.begin(), neighbor_values.end());
    if (min_threshold < *min_value || *max_value < max_threshold) {
      return;
    }

    // Determine the cell's position
    zero_crossings.clear();
    CHECK_GE(cell_index.height, 0);
    CHECK_LE(cell_index.height, max_height);
    const Point3D cell_center =
        convert::nodeIndexToCenterPoint(cell_index, min_cell_width);
    const FloatingPoint cell_width =
        convert::heightToCellWidth(min_cell_width, cell_index.height);

    // For each edge, identify where there is a sign change.
    auto interp = [](FloatingPoint a, FloatingPoint b) { return -a / (b - a); };
    for (auto dx : {0, 1}) {
      for (auto dy : {0, 1}) {
        const auto a =
            neighbor_values[convert::indexToLinearIndex<2, 3>({dx, dy, 0})];
        const auto b =
            neighbor_values[convert::indexToLinearIndex<2, 3>({dx, dy, 1})];
        if (std::min(a, b) < min_threshold && max_threshold < std::max(a, b)) {
          zero_crossings.emplace_back(
              Vector3D{cell_center.x() + (dx ? cell_width : 0.f),
                       cell_center.y() + (dy ? cell_width : 0.f),
                       cell_center.z() + interp(a, b)});
        }
      }
    }

    for (auto dx : {0, 1}) {
      for (auto dz : {0, 1}) {
        const auto a =
            neighbor_values[convert::indexToLinearIndex<2, 3>({dx, 0, dz})];
        const auto b =
            neighbor_values[convert::indexToLinearIndex<2, 3>({dx, 1, dz})];
        if (std::min(a, b) < min_threshold && max_threshold < std::max(a, b)) {
          zero_crossings.emplace_back(
              Vector3D{cell_center.x() + (dx ? cell_width : 0.f),
                       cell_center.y() + interp(a, b),
                       cell_center.z() + (dz ? cell_width : 0.f)});
        }
      }
    }

    for (auto dy : {0, 1}) {
      for (auto dz : {0, 1}) {
        const auto a =
            neighbor_values[convert::indexToLinearIndex<2, 3>({0, dy, dz})];
        const auto b =
            neighbor_values[convert::indexToLinearIndex<2, 3>({1, dy, dz})];
        if (std::min(a, b) < min_threshold && max_threshold < std::max(a, b)) {
          zero_crossings.emplace_back(
              Vector3D{cell_center.x() + interp(a, b),
                       cell_center.y() + (dy ? cell_width : 0.f),
                       cell_center.z() + (dz ? cell_width : 0.f)});
        }
      }
    }

    CHECK_LE(zero_crossings.size(), 12);
    if (zero_crossings.empty()) {
      return;
    }
    // TODO(victorr): Actually solve the QEF and add the resulting point
    mesh_object_->position(cell_center.x(), cell_center.y(), cell_center.z());
    Ogre::ColourValue color;
    const FloatingPoint cell_prob = log_odds_to_prob(cell_log_odds);
    color.r = cell_prob;
    color.g = cell_prob;
    color.b = cell_prob;
    color.a = 1.f;
    mesh_object_->colour(color);
    vertex_map[cell_index.position] = vertex_map.size();
  });

  map.forEachLeaf([=, &map, &vertex_map](const OctreeIndex& cell_index,
                                         FloatingPoint /*cell_log_odds*/) {
    if (cell_index.height != 0) {
      return;
    }

    const Index3D& pos_idx = cell_index.position;
    {
      const auto a = map.getCellValue({pos_idx.x(), pos_idx.y(), pos_idx.z()});
      const auto b =
          map.getCellValue({pos_idx.x(), pos_idx.y(), pos_idx.z() + 1});
      if (std::min(a, b) < min_threshold && max_threshold < std::max(a, b)) {
        try {
          std::array<Ogre::uint32, 4> quad_sides{
              vertex_map.at({pos_idx.x() - 1, pos_idx.y() - 1, pos_idx.z()}),
              vertex_map.at({pos_idx.x(), pos_idx.y() - 1, pos_idx.z()}),
              vertex_map.at({pos_idx.x(), pos_idx.y(), pos_idx.z()}),
              vertex_map.at({pos_idx.x() - 1, pos_idx.y(), pos_idx.z()})};
          if (max_threshold < b) {
            std::reverse(quad_sides.begin(), quad_sides.end());
          }
          mesh_object_->quad(quad_sides[0], quad_sides[1], quad_sides[2],
                             quad_sides[3]);
        } catch (const std::out_of_range&) {
        }
      }
    }

    {
      const auto a = map.getCellValue({pos_idx.x(), pos_idx.y(), pos_idx.z()});
      const auto b =
          map.getCellValue({pos_idx.x(), pos_idx.y() + 1, pos_idx.z()});
      if (std::min(a, b) < min_threshold && max_threshold < std::max(a, b)) {
        try {
          std::array<Ogre::uint32, 4> quad_sides{
              vertex_map.at({pos_idx.x() - 1, pos_idx.y(), pos_idx.z() - 1}),
              vertex_map.at({pos_idx.x(), pos_idx.y(), pos_idx.z() - 1}),
              vertex_map.at({pos_idx.x(), pos_idx.y(), pos_idx.z()}),
              vertex_map.at({pos_idx.x() - 1, pos_idx.y(), pos_idx.z()})};
          if (min_threshold < a) {
            std::reverse(quad_sides.begin(), quad_sides.end());
          }
          mesh_object_->quad(quad_sides[0], quad_sides[1], quad_sides[2],
                             quad_sides[3]);
        } catch (const std::out_of_range&) {
        }
      }
    }

    {
      const auto a = map.getCellValue({pos_idx.x(), pos_idx.y(), pos_idx.z()});
      const auto b =
          map.getCellValue({pos_idx.x() + 1, pos_idx.y(), pos_idx.z()});
      if (std::min(a, b) < min_threshold && max_threshold < std::max(a, b)) {
        try {
          std::array<Ogre::uint32, 4> quad_sides{
              vertex_map.at({pos_idx.x(), pos_idx.y() - 1, pos_idx.z() - 1}),
              vertex_map.at({pos_idx.x(), pos_idx.y(), pos_idx.z() - 1}),
              vertex_map.at({pos_idx.x(), pos_idx.y(), pos_idx.z()}),
              vertex_map.at({pos_idx.x(), pos_idx.y() - 1, pos_idx.z()})};
          if (max_threshold < b) {
            std::reverse(quad_sides.begin(), quad_sides.end());
          }
          mesh_object_->quad(quad_sides[0], quad_sides[1], quad_sides[2],
                             quad_sides[3]);
        } catch (const std::out_of_range&) {
        }
      }
    }
  });
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
