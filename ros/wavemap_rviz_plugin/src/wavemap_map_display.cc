#include "wavemap_rviz_plugin/wavemap_map_display.h"

#include <OGRE/OgreSceneNode.h>
#include <rviz/visualization_manager.h>
#include <tf/transform_listener.h>
#include <wavemap/indexing/ndtree_index.h>
#include <wavemap_ros_conversions/map_msg_conversions.h>

#include "wavemap_rviz_plugin/visuals/grid_visual.h"

namespace wavemap::rviz_plugin {
// After the top-level rviz::Display::initialize() does its own setup,
// it calls the subclass's onInitialize() function. This is where we
// instantiate all the workings of the class. We make sure to also
// call our immediate super-class's onInitialize() function, since it
// does important stuff setting up the message filter.
void WavemapMapDisplay::onInitialize() {
  MFDClass::onInitialize();
  grid_visual_ =
      std::make_unique<GridVisual>(context_->getSceneManager(), scene_node_,
                                   &grid_visual_properties_, map_mutex_, map_);
  slice_visual_ = std::make_unique<SliceVisual>(
      context_->getSceneManager(), scene_node_, &slice_visual_properties_,
      map_mutex_, map_);
}

// Clear the visuals by deleting their objects.
void WavemapMapDisplay::reset() {
  MFDClass::reset();
  //  grid_visual_.reset();
  //  slice_visual_.reset();
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
  //  grid_visual_.setFramePosition(position);
  //  grid_visual_.setFrameOrientation(orientation);
  //  slice_visual_.setFramePosition(position);
  //  slice_visual_.setFrameOrientation(orientation);

  // Update the multi-resolution grid and slice visual's contents if they exist
  if (std::shared_lock lock(*map_mutex_); !map_) {
    return;
  }

  //  // Update the visualizations
  //  grid_visual_.update();
  //  slice_visual_.update();
}

void WavemapMapDisplay::updateMapFromRosMsg(const wavemap_msgs::Map& map_msg) {
  std::unique_lock lock(*map_mutex_);
  convert::rosMsgToMap(map_msg, map_);
}
}  // namespace wavemap::rviz_plugin

// Tell pluginlib about this class.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(wavemap::rviz_plugin::WavemapMapDisplay, rviz::Display)
