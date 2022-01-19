#ifndef WAVEMAP_2D_GROUND_TRUTH_USER_INTERFACES_GAZEBO_PLUGIN_H_
#define WAVEMAP_2D_GROUND_TRUTH_USER_INTERFACES_GAZEBO_PLUGIN_H_

#include <string>
#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/physics.hh>
#include <glog/logging.h>
#include <ros/ros.h>
#include <wavemap_2d_msgs/FilePath.h>

#include "wavemap_2d/common.h"

namespace gazebo {
class Wavemap2DGroundTruthPlugin : public WorldPlugin {
 public:
  Wavemap2DGroundTruthPlugin();

  void Load(physics::WorldPtr world, sdf::ElementPtr _sdf) override;

  bool serviceCallback(wavemap_2d_msgs::FilePath::Request& request,
                       wavemap_2d_msgs::FilePath::Response& response);

 private:
  physics::WorldPtr world_;
  ros::NodeHandle nh_private_;
  ros::ServiceServer srv_;

  static std::string getMeshTypeStr(common::SubMesh::PrimitiveType mesh_type) {
    const std::vector<std::string> kMeshTypeStrs(
        {"POINTS", "LINES", "LINESTRIPS", "TRIANGLES", "TRIFANS", "TRISTRIPS"});
    return kMeshTypeStrs[mesh_type];
  }
  const std::vector<std::string> kMeshFileExtensions = {".dae", ".obj", ".mtl"};
};
}  // namespace gazebo

#endif  // WAVEMAP_2D_GROUND_TRUTH_USER_INTERFACES_GAZEBO_PLUGIN_H_
