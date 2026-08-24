#include "wavemap_rviz_plugin/wavemap_map_display.h"

#include <algorithm>
#include <memory>
#include <string>

#include <OGRE/OgreSceneNode.h>
#include <qfiledialog.h>
#include <QString>
#include <rviz/visualization_manager.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>
#include <wavemap/core/utils/profile/profiler_interface.h>
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

  // The layered map selector is only useful after receiving a layered map message.
  layer_property_.clearOptions();
  layer_property_.addOption(QString::fromStdString(selected_layer_name_));
  layer_property_.setStringStd(selected_layer_name_);
  layer_property_.setHidden(true);
}

// After the top-level rviz::Display::initialize() does its own setup,
// it calls the subclass's onInitialize() function. This is where we
// instantiate all the workings of the class. We make sure to also
// call our immediate super-class's onInitialize() function, since it
// does important stuff setting up the message filter.
void WavemapMapDisplay::onInitialize() {
  ProfilerZoneScoped;
  MFDClass::onInitialize();
  voxel_visual_ = std::make_unique<VoxelVisual>(
      scene_manager_, context_->getViewManager(), scene_node_,
      &voxel_visual_properties_, map_and_mutex_);
  slice_visual_ = std::make_unique<SliceVisual>(
      scene_manager_, scene_node_, &slice_visual_properties_, map_and_mutex_);
}

// Clear the visuals by deleting their objects.
void WavemapMapDisplay::reset() {
  ProfilerZoneScoped;
  MFDClass::reset();
  voxel_visual_->clear();
  slice_visual_->clear();
}

bool WavemapMapDisplay::hasMap() {
  ProfilerZoneScoped;
  std::scoped_lock lock(map_and_mutex_->mutex);
  return static_cast<bool>(map_and_mutex_->map) || static_cast<bool>(map_and_mutex_->layered_map) || map_and_mutex_->layered_map_msg.has_value();
}

void WavemapMapDisplay::clearMap() {
  ProfilerZoneScoped;
  std::scoped_lock lock(map_and_mutex_->mutex);
  if (map_and_mutex_->map) {
    map_and_mutex_->map->clear();
  }
  map_and_mutex_->layered_map.reset();
  map_and_mutex_->layered_map_msg.reset();
}

bool WavemapMapDisplay::loadMapFromDisk(const std::filesystem::path& filepath) {
  ProfilerZoneScoped;
  std::scoped_lock lock(map_and_mutex_->mutex);
  map_and_mutex_->layered_map.reset();
  map_and_mutex_->layered_map_msg.reset();
  return io::fileToMap(filepath, map_and_mutex_->map);
}

void WavemapMapDisplay::updateVisuals(bool redraw_all) {
  ProfilerZoneScoped;
  if (!hasMap()) {
    return;
  }
  voxel_visual_->updateMap(redraw_all);
  slice_visual_->update();
}

// This is our callback to handle an incoming message
void WavemapMapDisplay::processMessage(
    const wavemap_msgs::Map::ConstPtr& map_msg) {
  ProfilerZoneScoped;
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
  ProfilerZoneScoped;
  updateLayerMetadataFromRosMsg(map_msg);

  const bool has_layered_map = map_msg.layered_hashed_wavelet_octree.size() == 1u;
  const bool has_legacy_map = !map_msg.hashed_blocks.empty() || !map_msg.wavelet_octree.empty() || !map_msg.hashed_wavelet_octree.empty();

  std::scoped_lock lock(map_and_mutex_->mutex);
  map_and_mutex_->selected_layer_name = selected_layer_name_;
  if (has_layered_map && !has_legacy_map) {
    map_and_mutex_->map.reset();
    map_and_mutex_->layered_map.reset();
    map_and_mutex_->layered_map_msg = map_msg.layered_hashed_wavelet_octree.front();
    return;
  }

  map_and_mutex_->layered_map.reset();
  map_and_mutex_->layered_map_msg.reset();
  if (!convert::rosMsgToMap(map_msg, map_and_mutex_->map)) {
    ROS_WARN("Failed to parse map message.");
  }
}


void WavemapMapDisplay::updateLayerMetadataFromRosMsg(const wavemap_msgs::Map& map_msg) {
  ProfilerZoneScoped;
  available_layers_.clear();
  layer_property_.clearOptions();
  layer_property_.addOption("occupancy");

  if (map_msg.layered_hashed_wavelet_octree.empty()) {
    selected_layer_name_ = "occupancy";
    layer_property_.setStringStd(selected_layer_name_);
    layer_property_.setHidden(true);
    return;
  }

  const auto& layered_map_msg = map_msg.layered_hashed_wavelet_octree.front();
  const size_t num_layers = std::min(layered_map_msg.layer_names.size(), layered_map_msg.layer_types.size());
  for (size_t layer_idx = 0u; layer_idx < num_layers; ++layer_idx) {
    available_layers_.push_back({layered_map_msg.layer_names[layer_idx],
                                 layered_map_msg.layer_types[layer_idx],
                                 {}, {}});
    layer_property_.addOption(QString::fromStdString(layered_map_msg.layer_names[layer_idx]));
  }

  const bool selected_layer_still_exists =
      selected_layer_name_ == "occupancy" ||
      std::any_of(available_layers_.begin(), available_layers_.end(),
                  [this](const LayerMetadata& layer) {
                    return layer.name == selected_layer_name_;
                  });
  if (!selected_layer_still_exists) {
    selected_layer_name_ = "occupancy";
  }

  layer_property_.setStringStd(selected_layer_name_);
  layer_property_.setHidden(false);

  ROS_DEBUG_STREAM("Received layered map metadata with " << available_layers_.size() << " custom layers.");
}

void WavemapMapDisplay::updateLayerSelectionCallback() {
  ProfilerZoneScoped;
  selected_layer_name_ = layer_property_.getStdString();
  {
    std::scoped_lock lock(map_and_mutex_->mutex);
    map_and_mutex_->selected_layer_name = selected_layer_name_;
  }
  updateVisuals(true);
}

void WavemapMapDisplay::updateSourceModeCallback() {
  ProfilerZoneScoped;
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
  ProfilerZoneScoped;
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
  ProfilerZoneScoped;
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
  ProfilerZoneScoped;
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
  ProfilerZoneScoped;
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
