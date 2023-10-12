#include "wavemap_rviz_plugin/wavemap_map_display.h"

#include <OGRE/OgreSceneNode.h>
#include <qfiledialog.h>
#include <rviz/visualization_manager.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <wavemap/indexing/ndtree_index.h>
#include <wavemap_io/file_conversions.h>
#include <wavemap_ros_conversions/map_msg_conversions.h>

#include "wavemap_rviz_plugin/utils/alert_dialog.h"
#include "wavemap_rviz_plugin/visuals/grid_visual.h"

namespace wavemap::rviz_plugin {
WavemapMapDisplay::WavemapMapDisplay() {
  // Initialize the property menu
  source_mode_property_.clearOptions();
  for (const auto& name : SourceMode::names) {
    source_mode_property_.addOption(name);
  }
  source_mode_property_.setStringStd(source_mode_.toStr());
}

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
}

// Clear the visuals by deleting their objects.
void WavemapMapDisplay::reset() {
  MFDClass::reset();
  grid_visual_->clear();
  slice_visual_->clear();
}

bool WavemapMapDisplay::hasMap() {
  std::scoped_lock lock(map_and_mutex_->mutex);
  return static_cast<bool>(map_and_mutex_->map);
}

void WavemapMapDisplay::clearMap() {
  std::scoped_lock lock(map_and_mutex_->mutex);
  map_and_mutex_->map->clear();
}

bool WavemapMapDisplay::loadMapFromDisk(const std::filesystem::path& filepath) {
  std::scoped_lock lock(map_and_mutex_->mutex);
  return io::fileToMap(filepath, map_and_mutex_->map);
}

void WavemapMapDisplay::updateVisuals(bool redraw_all) {
  if (!hasMap()) {
    return;
  }
  grid_visual_->updateMap(redraw_all);
  slice_visual_->update();
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
  updateVisuals();
}

void WavemapMapDisplay::updateMapFromRosMsg(const wavemap_msgs::Map& map_msg) {
  std::scoped_lock lock(map_and_mutex_->mutex);
  if (!convert::rosMsgToMap(map_msg, map_and_mutex_->map)) {
    ROS_WARN("Failed to parse map message.");
  }
}

void WavemapMapDisplay::updateSourceModeCallback() {
  // Update the cached source mode value
  const SourceMode old_source_mode = source_mode_;
  source_mode_ = SourceMode(source_mode_property_.getStdString());

  // Show/hide the properties appropriate for mode kFromTopic
  unreliable_property_->setHidden(source_mode_ != SourceMode::kFromTopic);
  queue_size_property_->setHidden(source_mode_ != SourceMode::kFromTopic);
  topic_property_->setHidden(source_mode_ != SourceMode::kFromTopic);
  request_whole_map_property_.setHidden(source_mode_ != SourceMode::kFromTopic);

  // Show/hide the properties appropriate for mode kFromFile
  load_map_from_disk_property_.setHidden(source_mode_ != SourceMode::kFromFile);
  load_map_from_disk_property_.resetAllValues();

  // Update the map if the source mode changed
  if (source_mode_ != old_source_mode) {
    // Subscribe to the ROS topic if appropriate
    if (source_mode_ == SourceMode::kFromTopic) {
      updateTopic();
    } else {  // Otherwise, unsubscribe
      unsubscribe();
    }
    // Reset the map and update the visuals
    clearMap();
    updateVisuals(true);
  }
}

void WavemapMapDisplay::requestWholeMapCallback() {
  // Resolve name of the service based on the map topic
  const auto& map_topic = sub_.getTopic();
  const auto service_name = resolveMapUpdateServiceNameFromMapTopic(map_topic);
  if (service_name) {
    // If we managed to resolve the service name,
    // check if it differs from our current connection
    if (request_whole_map_client_.getService() != service_name.value()) {
      // If so, update it
      request_whole_map_client_ =
          ros::NodeHandle("wavemap_rviz_plugin")
              .serviceClient<std_srvs::Empty>(service_name.value());
    }
  } else {
    // If the service name could not be resolved,
    // make sure we don't stay connected to a service that's no longer relevant
    request_whole_map_client_.shutdown();

    // Alert the user that the service name could not be resolved
    AlertDialog alert{
        "Not available",
        "Could not resolve the wavemap_server's " +
            kRepublishWholeMapServiceSuffix +
            " service name from the map topic selected in Rviz. Does \"" +
            map_topic + "\" point to a wavemap_server's map topic?"};
    alert.exec();
    return;
  }

  // Check whether the service is available
  if (!request_whole_map_client_.exists()) {
    AlertDialog alert{"Not available",
                      "Could not connect to service:\n\"" +
                          service_name.value() +
                          "\".\nIs the wavemap_server running and is the map "
                          "topic selected in Rviz correct?"};
    alert.exec();
    return;
  }

  // Call the service
  std_srvs::Empty msg{};
  if (!request_whole_map_client_.call(msg)) {
    // Alert the user if it failed
    AlertDialog alert{
        "Error",
        "The service called:\n\"" + service_name.value() +
            "\"\nexists but could not be called. Is the wavemap_server "
            "running and is the map topic selected in Rviz correct?"};
    alert.exec();
  }
}

void WavemapMapDisplay::loadMapFromDiskCallback() {
  // Open file selection dialog
  const auto filepath_qt = QFileDialog::getOpenFileName();

  // Check if the chosen filepath is not empty
  if (filepath_qt.isEmpty()) {
    load_map_from_disk_property_.resetAllValues();
    return;
  }

  // Load the map
  const std::filesystem::path filepath{filepath_qt.toStdString()};
  if (!loadMapFromDisk(filepath)) {
    load_map_from_disk_property_.resetAllValues();
    return;
  }

  // Update the button property to show the map's name (when not in focus)
  load_map_from_disk_property_.setAtRestValue(filepath.filename());

  // Update the visuals
  updateVisuals(true);
}

std::optional<std::string>
WavemapMapDisplay::resolveMapUpdateServiceNameFromMapTopic(
    const std::string& map_topic) {
  const auto pos = map_topic.rfind('/');
  if (pos == std::string::npos) {
    return std::nullopt;
  }

  std::string service_topic =
      map_topic.substr(0, pos) + "/" + kRepublishWholeMapServiceSuffix;
  return service_topic;
}
}  // namespace wavemap::rviz_plugin

// Tell pluginlib about this class.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(wavemap::rviz_plugin::WavemapMapDisplay, rviz::Display)
