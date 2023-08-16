#include "wavemap_rviz_plugin/wavemap_map_display.h"

#include <OGRE/OgreSceneNode.h>
#include <rviz/visualization_manager.h>
#include <std_srvs/Empty.h>
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
  grid_visual_ = std::make_unique<GridVisual>(
      scene_manager_, context_->getViewManager(), scene_node_,
      &grid_visual_properties_, map_and_mutex_);
  slice_visual_ = std::make_unique<SliceVisual>(
      scene_manager_, scene_node_, &slice_visual_properties_, map_and_mutex_);
  request_whole_map_client_ =
      ros::NodeHandle("wavemap_rviz_plugin")
          .serviceClient<std_srvs::Empty>("/wavemap/republish_whole_map");
}

// Clear the visuals by deleting their objects.
void WavemapMapDisplay::reset() {
  MFDClass::reset();
  grid_visual_->clear();
  slice_visual_->clear();
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

  // Check that the visuals are initialized before continuing
  if (!grid_visual_ || !slice_visual_) {
    ROS_WARN("Visuals not initialized yet, skipping message.");
    return;
  }

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

  grid_visual_->setFramePosition(position);
  grid_visual_->setFrameOrientation(orientation);
  slice_visual_->setFramePosition(position);
  slice_visual_->setFrameOrientation(orientation);

  // Update the multi-resolution grid and slice visual's contents if they exist
  if (std::scoped_lock lock(map_and_mutex_->mutex); !map_and_mutex_->map) {
    return;
  }

  // Update the visualizations
  grid_visual_->updateMap();
  slice_visual_->update();
}

void WavemapMapDisplay::updateMapFromRosMsg(const wavemap_msgs::Map& map_msg) {
  std::scoped_lock lock(map_and_mutex_->mutex);
  if (!convert::rosMsgToMap(map_msg, map_and_mutex_->map)) {
    ROS_WARN("Failed to parse map message.");
  }
}

void WavemapMapDisplay::requestWholeMap() {
  if (request_whole_map_property_.getBool()) {
    std_srvs::Empty msg{};
    if (request_whole_map_client_.call(msg)) {
      request_whole_map_property_.setBool(false);
    } else {
      ROS_WARN_STREAM("Failed to call "
                      << request_whole_map_client_.getService() << ".");
    }
  }
}
}  // namespace wavemap::rviz_plugin

// Tell pluginlib about this class.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(wavemap::rviz_plugin::WavemapMapDisplay, rviz::Display)
