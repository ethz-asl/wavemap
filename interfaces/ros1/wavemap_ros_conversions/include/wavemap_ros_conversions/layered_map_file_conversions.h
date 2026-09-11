#ifndef WAVEMAP_ROS_CONVERSIONS_LAYERED_MAP_FILE_CONVERSIONS_H_
#define WAVEMAP_ROS_CONVERSIONS_LAYERED_MAP_FILE_CONVERSIONS_H_

#include <filesystem>
#include <string>

#include <ros/time.h>
#include <wavemap_msgs/LayeredMap.h>

namespace wavemap::convert {

// Loads a version-1 .lwvmp file by interpreting the layer schema stored in the
// file. This path supports Wavemap's built-in stream codecs and does not
// require the original compile-time LayerSchema to be present in the process.
bool layeredMapFileToRosMsg(const std::filesystem::path& file_path,
                            const std::string& frame_id,
                            const ros::Time& stamp,
                            wavemap_msgs::LayeredMap& msg,
                            std::string* error_message = nullptr);

}  // namespace wavemap::convert

#endif  // WAVEMAP_ROS_CONVERSIONS_LAYERED_MAP_FILE_CONVERSIONS_H_
