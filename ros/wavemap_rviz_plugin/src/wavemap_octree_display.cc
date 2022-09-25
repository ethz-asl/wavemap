#include "wavemap_rviz_plugin/wavemap_octree_display.h"

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>
#include <tf/transform_listener.h>

#include "wavemap_rviz_plugin/wavemap_octree_visual.h"

namespace wavemap_rviz_plugin {
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
WavemapOctreeDisplay::WavemapOctreeDisplay() {
  occupancy_threshold_property_ =
      new rviz::FloatProperty("Occupancy threshold log odds", 0.0,
                              "A 50% occupancy probability corresponds to 0.0.",
                              this, SLOT(updateOccupancyThreshold()));
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
  visual_.reset();
}

// Update the visual with the current occupancy threshold
void WavemapOctreeDisplay::updateOccupancyThreshold() {
  const float occupancy_threshold_log_odds =
      occupancy_threshold_property_->getFloat();

  visual_->setOccupancyThreshold(occupancy_threshold_log_odds);
}

// This is our callback to handle an incoming message
void WavemapOctreeDisplay::processMessage(
    const wavemap_msgs::Octree::ConstPtr& msg) {
  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this WavemapOctree message. If
  // it fails, we can't do anything else, so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(
          msg->header.frame_id, msg->header.stamp, position, orientation)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  // Create the visual if it does not yet exist
  if (!visual_) {
    visual_ = std::make_unique<WavemapOctreeVisual>(context_->getSceneManager(),
                                                    scene_node_);
  }

  // Set or update the contents of the chosen visual
  visual_->setMessage(msg);
  visual_->setFramePosition(position);
  visual_->setFrameOrientation(orientation);

  float occupancy_threshold_log_odds =
      occupancy_threshold_property_->getFloat();
  visual_->setOccupancyThreshold(occupancy_threshold_log_odds);
}

}  // namespace wavemap_rviz_plugin

// Tell pluginlib about this class.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(wavemap_rviz_plugin::WavemapOctreeDisplay, rviz::Display)
