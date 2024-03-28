#include "wavemap_rviz_plugin/wavemap_map_display.h"

#include <OGRE/OgreSceneNode.h>
#include <qfiledialog.h>
#include <rviz/visualization_manager.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>
#include <tracy/Tracy.hpp>
#include <wavemap/io/file_conversions.h>
#include <wavemap_ros_conversions/map_msg_conversions.h>

#include "wavemap_rviz_plugin/utils/alert_dialog.h"

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
  ZoneScoped;
  MFDClass::onInitialize();
  voxel_visual_ = std::make_unique<VoxelVisual>(
      scene_manager_, context_->getViewManager(), scene_node_,
      &voxel_visual_properties_, map_and_mutex_);
  slice_visual_ = std::make_unique<SliceVisual>(
      scene_manager_, scene_node_, &slice_visual_properties_, map_and_mutex_);
}

// Clear the visuals by deleting their objects.
void WavemapMapDisplay::reset() {
  ZoneScoped;
  MFDClass::reset();
  voxel_visual_->clear();
  slice_visual_->clear();
}

bool WavemapMapDisplay::hasMap() {
  ZoneScoped;
  std::scoped_lock lock(map_and_mutex_->mutex);
  return static_cast<bool>(map_and_mutex_->map);
}

void WavemapMapDisplay::clearMap() {
  ZoneScoped;
  std::scoped_lock lock(map_and_mutex_->mutex);
  if (map_and_mutex_->map) {
    map_and_mutex_->map->clear();
  }
}

bool WavemapMapDisplay::loadMapFromDisk(const std::filesystem::path& filepath) {
  ZoneScoped;
  std::scoped_lock lock(map_and_mutex_->mutex);
  return io::fileToMap(filepath, map_and_mutex_->map);
}

void WavemapMapDisplay::updateVisuals(bool redraw_all) {
  ZoneScoped;
  if (!hasMap()) {
    return;
  }
  voxel_visual_->updateMap(redraw_all);
  slice_visual_->update();
}

// This is our callback to handle an incoming message
void WavemapMapDisplay::processMessage(
    const wavemap_msgs::Map::ConstPtr& map_msg) {
  ZoneScoped;
  // Deserialize the octree
  if (!map_msg) {
    ROS_WARN("Ignoring request to process non-existent octree msg (nullptr).");
    return;
  }
  updateMapFromRosMsg(*map_msg);

  // Check that the visuals are initialized before continuing
  if (!voxel_visual_ || !slice_visual_) {
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
  voxel_visual_->setFramePosition(position);
  voxel_visual_->setFrameOrientation(orientation);
  slice_visual_->setFramePosition(position);
  slice_visual_->setFrameOrientation(orientation);

  // Update the voxel and slice visual's contents if they exist
  updateVisuals();
}

void WavemapMapDisplay::updateMapFromRosMsg(const wavemap_msgs::Map& map_msg) {
  ZoneScoped;
  std::scoped_lock lock(map_and_mutex_->mutex);
  if (!convert::rosMsgToMap(map_msg, map_and_mutex_->map)) {
    ROS_WARN("Failed to parse map message.");
  }
}

void WavemapMapDisplay::updateSourceModeCallback() {
  ZoneScoped;
  // Update the cached source mode value
  const SourceMode old_source_mode = source_mode_;
  source_mode_ = SourceMode(source_mode_property_.getStdString());

  // Show/hide the properties appropriate for mode kFromTopic
  unreliable_property_->setHidden(source_mode_ != SourceMode::kFromTopic);
  queue_size_property_->setHidden(source_mode_ != SourceMode::kFromTopic);
  topic_property_->setHidden(source_mode_ != SourceMode::kFromTopic);
  request_whole_map_property_.setHidden(source_mode_ != SourceMode::kFromTopic);
  request_wavemap_server_reset_property_.setHidden(source_mode_ !=
                                                   SourceMode::kFromTopic);

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

void WavemapMapDisplay::requestWavemapServerResetCallback() {
  ZoneScoped;
  // Resolve name of the service based on the map topic
  const auto& map_topic = sub_.getTopic();
  const auto service_name = resolveWavemapServerNamespaceFromMapTopic(
      map_topic, kResetWavemapServerService);
  if (service_name) {
    // If we managed to resolve the service name,
    // check if it differs from our current connection
    if (request_wavemap_server_reset_client_.getService() !=
        service_name.value()) {
      // If so, update it
      request_wavemap_server_reset_client_ =
          ros::NodeHandle("wavemap_rviz_plugin")
              .serviceClient<std_srvs::Trigger>(service_name.value());
    }
  } else {
    // If the service name could not be resolved,
    // make sure we don't stay connected to a service that's no longer relevant
    request_wavemap_server_reset_client_.shutdown();

    // Alert the user that the service name could not be resolved
    AlertDialog alert{"Not available",
                      "Could not resolve the wavemap_server's namespace from "
                      "the map topic selected in Rviz. Does \"" +
                          map_topic +
                          "\" point to a wavemap_server's map topic?"};
    alert.exec();
    return;
  }

  // Check whether the service is available
  if (!request_wavemap_server_reset_client_.exists()) {
    AlertDialog alert{"Not available",
                      "Could not connect to service:\n\"" +
                          service_name.value() +
                          "\".\nIs the wavemap_server running and is the map "
                          "topic selected in Rviz correct?"};
    alert.exec();
    return;
  }

  // Call the service
  std_srvs::Trigger msg{};
  if (request_wavemap_server_reset_client_.call(msg)) {
    if (!msg.response.success) {
      // Alert the user if the call succeeded but the action did not
      AlertDialog alert{"Error", "Service:\n\"" + service_name.value() +
                                     "\"\nresponded \"" + msg.response.message +
                                     "\"."};
      alert.exec();
    }
  } else {
    // Alert the user if the call failed
    AlertDialog alert{
        "Error",
        "The service called:\n\"" + service_name.value() +
            "\"\nexists but could not be called. Is the wavemap_server "
            "running and is the map topic selected in Rviz correct?"};
    alert.exec();
  }
}

void WavemapMapDisplay::requestWholeMapCallback() {
  ZoneScoped;
  // Resolve name of the service based on the map topic
  const auto& map_topic = sub_.getTopic();
  const auto service_name = resolveWavemapServerNamespaceFromMapTopic(
      map_topic, kRepublishWholeMapService);
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
    AlertDialog alert{"Not available",
                      "Could not resolve the wavemap_server's namespace from "
                      "the map topic selected in Rviz. Does \"" +
                          map_topic +
                          "\" point to a wavemap_server's map topic?"};
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
  ZoneScoped;
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
WavemapMapDisplay::resolveWavemapServerNamespaceFromMapTopic(
    const std::string& map_topic, const std::string& child_topic) {
  ZoneScoped;
  const auto pos = map_topic.rfind('/');
  if (pos == std::string::npos) {
    return std::nullopt;
  }

  std::string wavemap_server_namespace = map_topic.substr(0, pos);
  if (child_topic.empty()) {
    return wavemap_server_namespace;
  } else {
    return wavemap_server_namespace + "/" + child_topic;
  }
}
}  // namespace wavemap::rviz_plugin

// Tell pluginlib about this class.
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(wavemap::rviz_plugin::WavemapMapDisplay, rviz::Display)
