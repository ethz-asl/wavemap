#include "wavemap_rviz_plugin/wavemap_map_display.h"

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>
#include <tf/transform_listener.h>
#include <wavemap/data_structure/volumetric/hashed_wavelet_octree.h>
#include <wavemap/data_structure/volumetric/volumetric_octree.h>
#include <wavemap/data_structure/volumetric/wavelet_octree.h>
#include <wavemap/indexing/ndtree_index.h>

#include "wavemap_rviz_plugin/visuals/multi_resolution_grid_visual.h"

namespace wavemap::rviz_plugin {
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
WavemapMapDisplay::WavemapMapDisplay() {
  min_occupancy_threshold_property_ = std::make_unique<rviz::FloatProperty>(
      "Min occupancy log odds", 1e-6, "Ranges from -Inf to Inf.", this,
      SLOT(updateOccupancyThresholdsOrOpacity()));
  max_occupancy_threshold_property_ = std::make_unique<rviz::FloatProperty>(
      "Max occupancy log odds", 1e6, "Ranges from -Inf to Inf.", this,
      SLOT(updateOccupancyThresholdsOrOpacity()));

  multi_resolution_grid_visibility_property_ =
      std::make_unique<rviz::BoolProperty>(
          "Show multi-res grid", true,
          "Whether to show the octree as a multi-resolution grid.", this,
          SLOT(updateMultiResolutionGridVisibility()));
  multi_resolution_slice_visibility_property_ =
      std::make_unique<rviz::BoolProperty>(
          "Show multi-res slice", false,
          "Whether to show the octree as a multi-resolution grid.", this,
          SLOT(updateMultiResolutionSliceVisibility()));
  mesh_visibility_property_ = std::make_unique<rviz::BoolProperty>(
      "Show mesh", false, "Whether to show the isosurface as a mesh.", this,
      SLOT(updateMeshVisibility()));

  multi_resolution_slice_height_property_ =
      std::make_unique<rviz::FloatProperty>(
          "Slice height", 0.0, "Z-coordinate of the map slice to display.",
          this, SLOT(updateMultiResolutionSliceHeight()));

  opacity_property_ = std::make_unique<rviz::FloatProperty>(
      "Alpha", 1.0, "Opacity of the displayed visuals.", this,
      SLOT(updateOccupancyThresholdsOrOpacity()));
}

// After the top-level rviz::Display::initialize() does its own setup,
// it calls the subclass's onInitialize() function. This is where we
// instantiate all the workings of the class. We make sure to also
// call our immediate super-class's onInitialize() function, since it
// does important stuff setting up the message filter.
void WavemapMapDisplay::onInitialize() { MFDClass::onInitialize(); }

// Clear the visuals by deleting their objects.
void WavemapMapDisplay::reset() {
  MFDClass::reset();
  multi_resolution_grid_visual_.reset();
  multi_resolution_slice_visual_.reset();
  mesh_visual_.reset();
}

// Update the visuals with the current occupancy thresholds
void WavemapMapDisplay::updateOccupancyThresholdsOrOpacity() {
  // Get the properties set in the UI
  const FloatingPoint min_occupancy_threshold =
      min_occupancy_threshold_property_->getFloat();
  const FloatingPoint max_occupancy_threshold =
      max_occupancy_threshold_property_->getFloat();
  const FloatingPoint slice_height =
      multi_resolution_slice_height_property_->getFloat();
  const FloatingPoint alpha = opacity_property_->getFloat();

  // Update the visuals (if they are enabled)
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (map_) {
    if (multi_resolution_grid_visual_) {
      multi_resolution_grid_visual_->loadMap(*map_, min_occupancy_threshold,
                                             max_occupancy_threshold, alpha);
    }
    if (multi_resolution_slice_visual_) {
      multi_resolution_slice_visual_->loadMap(*map_, min_occupancy_threshold,
                                              max_occupancy_threshold,
                                              slice_height, alpha);
    }
    if (mesh_visual_) {
      mesh_visual_->loadMap(*map_, min_occupancy_threshold,
                            max_occupancy_threshold, alpha);
    }
  }
}

// Toggle whether each visual exists (is enabled)
void WavemapMapDisplay::updateMultiResolutionGridVisibility() {
  const bool enable = multi_resolution_grid_visibility_property_->getBool();
  if (enable) {
    // Create the grid visual if it does not yet exist
    if (!multi_resolution_grid_visual_) {
      multi_resolution_grid_visual_ =
          std::make_unique<MultiResolutionGridVisual>(
              context_->getSceneManager(), scene_node_);
    }
  } else {
    // Delete the grid visual
    multi_resolution_grid_visual_.reset();
  }
}

void WavemapMapDisplay::updateMultiResolutionSliceVisibility() {
  const bool enable = multi_resolution_slice_visibility_property_->getBool();
  if (enable) {
    // Create the grid visual if it does not yet exist
    if (!multi_resolution_slice_visual_) {
      multi_resolution_slice_visual_ =
          std::make_unique<MultiResolutionSliceVisual>(
              context_->getSceneManager(), scene_node_);
    }
  } else {
    // Delete the grid visual
    multi_resolution_slice_visual_.reset();
  }
}

void WavemapMapDisplay::updateMeshVisibility() {
  const bool enable = mesh_visibility_property_->getBool();
  if (enable) {
    // Create the mesh visual if it does not yet exist
    if (!mesh_visual_) {
      mesh_visual_ = std::make_unique<MeshVisual>(context_->getSceneManager(),
                                                  scene_node_);
    }
  } else {
    // Delete the mesh visual
    mesh_visual_.reset();
  }
}

void WavemapMapDisplay::updateMultiResolutionSliceHeight() {
  const FloatingPoint min_occupancy_threshold =
      min_occupancy_threshold_property_->getFloat();
  const FloatingPoint max_occupancy_threshold =
      max_occupancy_threshold_property_->getFloat();
  const FloatingPoint slice_height =
      multi_resolution_slice_height_property_->getFloat();
  const FloatingPoint alpha = opacity_property_->getFloat();

  // Update the visuals (if they are enabled)
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (map_ && multi_resolution_slice_visual_) {
    multi_resolution_slice_visual_->loadMap(*map_, min_occupancy_threshold,
                                            max_occupancy_threshold,
                                            slice_height, alpha);
  }
}

// This is our callback to handle an incoming message
void WavemapMapDisplay::processMessage(
    const wavemap_msgs::Map::ConstPtr& map_msg) {
  // Deserialize the octree
  if (!map_msg) {
    ROS_WARN("Ignoring request to process non-existent octree msg (nullptr).");
    return;
  }
  updateMapFromRosMsg(*map_msg);

  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this WavemapOctree message. If
  // it fails, we can't do anything else, so we return.
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(map_msg->header.frame_id,
                                                 map_msg->header.stamp,
                                                 position, orientation)) {
    ROS_WARN("Error transforming from frame '%s' to frame '%s'",
             map_msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  // Make sure the visibility of the visuals is up-to-date
  updateMultiResolutionGridVisibility();
  updateMultiResolutionSliceVisibility();

  // Update the multi-resolution grid and slice visual's contents if they exist
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (!map_) {
    return;
  }
  const FloatingPoint min_occupancy_threshold =
      min_occupancy_threshold_property_->getFloat();
  const FloatingPoint max_occupancy_threshold =
      max_occupancy_threshold_property_->getFloat();
  const FloatingPoint alpha = opacity_property_->getFloat();
  if (multi_resolution_grid_visual_) {
    multi_resolution_grid_visual_->setFramePosition(position);
    multi_resolution_grid_visual_->setFrameOrientation(orientation);
    multi_resolution_grid_visual_->loadMap(*map_, min_occupancy_threshold,
                                           max_occupancy_threshold, alpha);
  }
  if (multi_resolution_slice_visual_) {
    const FloatingPoint slice_height =
        multi_resolution_slice_height_property_->getFloat();
    multi_resolution_slice_visual_->setFramePosition(position);
    multi_resolution_slice_visual_->setFrameOrientation(orientation);
    multi_resolution_slice_visual_->loadMap(*map_, min_occupancy_threshold,
                                            max_occupancy_threshold,
                                            slice_height, alpha);
  }
  if (mesh_visual_) {
    mesh_visual_->setFramePosition(position);
    mesh_visual_->setFrameOrientation(orientation);
    mesh_visual_->loadMap(*map_, min_occupancy_threshold,
                          max_occupancy_threshold, alpha);
  }
}

void WavemapMapDisplay::updateMapFromRosMsg(const wavemap_msgs::Map& map_msg) {
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (!map_msg.wavelet_octree.empty()) {
    if (map_msg.wavelet_octree.size() == 1) {
      auto wavelet_octree = std::dynamic_pointer_cast<WaveletOctree>(map_);
      if (wavelet_octree) {
        wavelet_octree->clear();
      } else {
        VolumetricDataStructureConfig config;
        config.min_cell_width = map_msg.wavelet_octree.front().min_cell_width;
        wavelet_octree = std::make_shared<WaveletOctree>(config);
        map_ = wavelet_octree;
      }

      const auto& wavelet_octree_msg = map_msg.wavelet_octree.front();
      wavelet_octree->getRootScale() =
          wavelet_octree_msg.root_node_scale_coefficient;

      std::stack<WaveletOctree::NodeType*> stack;
      stack.emplace(&wavelet_octree->getRootNode());
      for (const auto& node_msg : wavelet_octree_msg.nodes) {
        CHECK(!stack.empty());
        WaveletOctree::NodeType* current_node = stack.top();
        CHECK_NOTNULL(current_node);
        stack.pop();

        std::copy(node_msg.detail_coefficients.cbegin(),
                  node_msg.detail_coefficients.cend(),
                  current_node->data().begin());
        for (int relative_child_idx = wavemap::OctreeIndex::kNumChildren - 1;
             0 <= relative_child_idx; --relative_child_idx) {
          const bool child_exists =
              node_msg.allocated_children_bitset & (1 << relative_child_idx);
          if (child_exists) {
            stack.emplace(current_node->allocateChild(relative_child_idx));
          }
        }
      }
    } else {
      auto hashed_wavelet_octree =
          std::dynamic_pointer_cast<HashedWaveletOctree>(map_);
      if (!hashed_wavelet_octree) {
        VolumetricDataStructureConfig config;
        config.min_cell_width = map_msg.wavelet_octree.front().min_cell_width;
        hashed_wavelet_octree = std::make_shared<HashedWaveletOctree>(config);
        map_ = hashed_wavelet_octree;
      }

      for (const auto& block_msg : map_msg.wavelet_octree) {
        const Index3D block_index{block_msg.root_node_offset[0],
                                  block_msg.root_node_offset[1],
                                  block_msg.root_node_offset[2]};
        CHECK(!block_index.hasNaN()) << block_index;
        auto& block = hashed_wavelet_octree->getOrAllocateBlock(block_index);

        block.getRootScale() = block_msg.root_node_scale_coefficient;

        std::stack<WaveletOctree::NodeType*> stack;
        stack.emplace(&block.getRootNode());
        for (const auto& node_msg : block_msg.nodes) {
          CHECK(!stack.empty());
          WaveletOctree::NodeType* current_node = stack.top();
          CHECK_NOTNULL(current_node);
          stack.pop();

          std::copy(node_msg.detail_coefficients.cbegin(),
                    node_msg.detail_coefficients.cend(),
                    current_node->data().begin());
          for (int relative_child_idx = wavemap::OctreeIndex::kNumChildren - 1;
               0 <= relative_child_idx; --relative_child_idx) {
            const bool child_exists =
                node_msg.allocated_children_bitset & (1 << relative_child_idx);
            if (child_exists) {
              stack.emplace(current_node->allocateChild(relative_child_idx));
            }
          }
        }
      }
    }
  } else if (!map_msg.octree.empty()) {
    auto octree = std::dynamic_pointer_cast<VolumetricOctree>(map_);
    if (octree) {
      octree->clear();
    } else {
      VolumetricDataStructureConfig config;
      config.min_cell_width = map_msg.wavelet_octree.front().min_cell_width;
      octree = std::make_shared<VolumetricOctree>(config);
      map_ = octree;
    }

    const auto& octree_msg = map_msg.octree.front();
    std::stack<VolumetricOctree::NodeType*> stack;
    stack.emplace(&octree->getRootNode());
    for (const auto& node_msg : octree_msg.nodes) {
      CHECK(!stack.empty());
      VolumetricOctree::NodeType* current_node = stack.top();
      CHECK_NOTNULL(current_node);
      stack.pop();

      current_node->data() = node_msg.node_value;
      for (int relative_child_idx = wavemap::OctreeIndex::kNumChildren - 1;
           0 <= relative_child_idx; --relative_child_idx) {
        const bool child_exists =
            node_msg.allocated_children_bitset & (1 << relative_child_idx);
        if (child_exists) {
          stack.emplace(current_node->allocateChild(relative_child_idx));
        }
      }
    }
  }
}
}  // namespace wavemap::rviz_plugin

// Tell pluginlib about this class.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(wavemap::rviz_plugin::WavemapMapDisplay, rviz::Display)
