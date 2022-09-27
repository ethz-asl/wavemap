#include "wavemap_rviz_plugin/wavemap_octree_display.h"

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>
#include <tf/transform_listener.h>
#include <wavemap_common/indexing/ndtree_index.h>

#include "wavemap_rviz_plugin/multi_resolution_grid_visual.h"

namespace wavemap::rviz_plugin {
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
WavemapOctreeDisplay::WavemapOctreeDisplay() {
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
void WavemapOctreeDisplay::onInitialize() { MFDClass::onInitialize(); }

// Clear the visuals by deleting their objects.
void WavemapOctreeDisplay::reset() {
  MFDClass::reset();
  multi_resolution_grid_visual_.reset();
  multi_resolution_slice_visual_.reset();
}

// Update the visuals with the current occupancy thresholds
void WavemapOctreeDisplay::updateOccupancyThresholdsOrOpacity() {
  // Get the properties set in the UI
  const FloatingPoint min_occupancy_threshold =
      min_occupancy_threshold_property_->getFloat();
  const FloatingPoint max_occupancy_threshold =
      max_occupancy_threshold_property_->getFloat();
  const FloatingPoint slice_height =
      multi_resolution_slice_height_property_->getFloat();
  const FloatingPoint alpha = opacity_property_->getFloat();

  // Update the visuals (if they are enabled)
  if (octree_ && multi_resolution_grid_visual_) {
    multi_resolution_grid_visual_->setOctree(*octree_, min_occupancy_threshold,
                                             max_occupancy_threshold, alpha);
  }
  if (octree_ && multi_resolution_slice_visual_) {
    multi_resolution_slice_visual_->setOctree(*octree_, min_occupancy_threshold,
                                              max_occupancy_threshold,
                                              slice_height, alpha);
  }
}

// Toggle whether each visual exists (is enabled)
void WavemapOctreeDisplay::updateMultiResolutionGridVisibility() {
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

void WavemapOctreeDisplay::updateMultiResolutionSliceVisibility() {
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

void WavemapOctreeDisplay::updateMultiResolutionSliceHeight() {
  const FloatingPoint min_occupancy_threshold =
      min_occupancy_threshold_property_->getFloat();
  const FloatingPoint max_occupancy_threshold =
      max_occupancy_threshold_property_->getFloat();
  const FloatingPoint slice_height =
      multi_resolution_slice_height_property_->getFloat();
  const FloatingPoint alpha = opacity_property_->getFloat();

  // Update the visuals (if they are enabled)
  if (octree_ && multi_resolution_slice_visual_) {
    multi_resolution_slice_visual_->setOctree(*octree_, min_occupancy_threshold,
                                              max_occupancy_threshold,
                                              slice_height, alpha);
  }
}

// This is our callback to handle an incoming message
void WavemapOctreeDisplay::processMessage(
    const wavemap_msgs::Octree::ConstPtr& octree_msg) {
  // Deserialize the octree
  if (!octree_msg) {
    ROS_WARN("Ignoring request to process non-existent octree msg (nullptr).");
    return;
  }
  octree_ = octreeFromRosMsg(*octree_msg);

  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this WavemapOctree message. If
  // it fails, we can't do anything else, so we return.
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(octree_msg->header.frame_id,
                                                 octree_msg->header.stamp,
                                                 position, orientation)) {
    ROS_WARN("Error transforming from frame '%s' to frame '%s'",
             octree_msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  // Make sure the visibility of the visuals is up-to-date
  updateMultiResolutionGridVisibility();
  updateMultiResolutionSliceVisibility();

  // Update the multi-resolution grid and slice visual's contents if they exist
  const FloatingPoint min_occupancy_threshold =
      min_occupancy_threshold_property_->getFloat();
  const FloatingPoint max_occupancy_threshold =
      max_occupancy_threshold_property_->getFloat();
  const FloatingPoint alpha = opacity_property_->getFloat();
  if (multi_resolution_grid_visual_) {
    multi_resolution_grid_visual_->setFramePosition(position);
    multi_resolution_grid_visual_->setFrameOrientation(orientation);
    multi_resolution_grid_visual_->setOctree(*octree_, min_occupancy_threshold,
                                             max_occupancy_threshold, alpha);
  }
  if (multi_resolution_slice_visual_) {
    const FloatingPoint slice_height =
        multi_resolution_slice_height_property_->getFloat();
    multi_resolution_slice_visual_->setFramePosition(position);
    multi_resolution_slice_visual_->setFrameOrientation(orientation);
    multi_resolution_slice_visual_->setOctree(*octree_, min_occupancy_threshold,
                                              max_occupancy_threshold,
                                              slice_height, alpha);
  }
}

std::unique_ptr<Octree> WavemapOctreeDisplay::octreeFromRosMsg(
    const wavemap_msgs::Octree& octree_msg) {
  auto octree = std::make_unique<Octree>(octree_msg.min_cell_width);

  std::stack<Octree::NodeType*> stack;
  stack.template emplace(&octree->getRootNode());
  for (const auto& node_msg : octree_msg.nodes) {
    CHECK(!stack.empty());
    Octree::NodeType* current_node = stack.top();
    stack.pop();

    current_node->data() = node_msg.node_value;
    for (int relative_child_idx = wavemap::OctreeIndex::kNumChildren - 1;
         0 <= relative_child_idx; --relative_child_idx) {
      const bool child_exists =
          node_msg.allocated_children_bitset & (1 << relative_child_idx);
      if (child_exists) {
        stack.template emplace(current_node->allocateChild(relative_child_idx));
      }
    }
  }

  return octree;
}
}  // namespace wavemap::rviz_plugin

// Tell pluginlib about this class.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(wavemap::rviz_plugin::WavemapOctreeDisplay,
                       rviz::Display)
