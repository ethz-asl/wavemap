#ifndef WAVEMAP_ROS_LAYERED_ROS_SERVER_EXTENSION_H_
#define WAVEMAP_ROS_LAYERED_ROS_SERVER_EXTENSION_H_

#include <filesystem>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <wavemap/layered/integration/layered_pipeline.h>
#include <wavemap/layered/integration/continuous_layer_updater.h>
#include <wavemap/layered/integration/layer_integrator.h>
#include <wavemap/layered/integration/layer_observation_batch.h>
#include <wavemap/layered/integration/endpoint_range.h>
#include <wavemap/layered/integration/pointcloud_layer_integration_mode.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap_msgs/FilePath.h>
#include <wavemap_msgs/LayeredMap.h>
#include <wavemap_msgs/LayeredMapUpdate.h>
#include <wavemap_ros_conversions/descriptor_layer_observation_conversions.h>
#include <wavemap_ros_conversions/layered_map_msg_conversions.h>

namespace wavemap {
class LayeredRosServerExtensionBase {
 public:
  virtual ~LayeredRosServerExtensionBase() = default;

  virtual bool saveLayeredMap(const std::filesystem::path& file_path) = 0;
  virtual bool loadLayeredMap(const std::filesystem::path& file_path,
                              std::string* error_message = nullptr) = 0;
  virtual bool publishLayeredMap(const ros::Time& stamp = ros::Time::now()) = 0;
  virtual MapBase::Ptr getContinuousMap() = 0;
  virtual void clearLayeredMap() = 0;
};

template <
    typename LayeredMapT, typename LayeredMapIoT, typename RosConverterT,
    typename UpdateConverterT =
        convert::DescriptorLayerObservationRosConverter<LayeredMapT>>
class LayeredRosServerExtension : public LayeredRosServerExtensionBase {
 public:
  LayeredRosServerExtension(ros::NodeHandle nh_private, std::string world_frame,
                            LayeredMapT layered_map,
                            std::string layered_map_topic = "layered_map",
                            std::string update_topic = "layered_map_updates")
      : nh_private_(std::move(nh_private)),
        world_frame_(std::move(world_frame)),
        layered_map_(std::move(layered_map)),
        layered_pipeline_(layered_map_) {
    layered_map_pub_ = nh_private_.advertise<wavemap_msgs::LayeredMap>(
        layered_map_topic, kQueueSize, kLatchPublisher);
    layered_update_sub_ = nh_private_.subscribe(
        update_topic, kQueueSize,
        &LayeredRosServerExtension::layeredUpdateCallback, this);
    advertiseServices();
  }

  LayeredMapT& getLayeredMap() { return layered_map_; }
  const LayeredMapT& getLayeredMap() const { return layered_map_; }
  MapBase::Ptr getContinuousMap() override {
    return layered_map_.continuousMapPtr();
  }
  void clearLayeredMap() override {
    std::scoped_lock lock(mutex_);
    layered_map_.continuousMap().clear();
    layered::forEachLayerDescriptor(
        layered_map_.discreteLayers(),
        [](const auto& /*descriptor*/, auto& layer) { layer.clear(); });
  }

  template <typename LayerTagT>
  bool integrateEndpointLayer(const PosedPointcloud<>& pointcloud,
                              const std::vector<
                                  layered::LayerValueT<LayerTagT>>& values,
                              const layered::EndpointRange& endpoint_range = {}) {
    return integratePointcloudLayer<
        LayerTagT,
        layered::PointcloudLayerIntegrationMode::kEndpointOnly>(
        pointcloud, values, endpoint_range);
  }

  template <typename LayerTagT,
            layered::PointcloudLayerIntegrationMode IntegrationMode>
  bool integratePointcloudLayer(
      const PosedPointcloud<>& pointcloud,
      const std::vector<layered::LayerValueT<LayerTagT>>& values,
      const layered::EndpointRange& endpoint_range = {}) {
    return integratePointcloudLayerImpl<LayerTagT, IntegrationMode>(
        pointcloud, values.size(),
        [&values](size_t index)
            -> std::optional<layered::LayerValueT<LayerTagT>> {
          return values[index];
        },
        endpoint_range);
  }

  template <typename LayerTagT>
  bool integrateOptionalEndpointLayer(
      const PosedPointcloud<>& pointcloud,
      const std::vector<std::optional<
          layered::LayerValueT<LayerTagT>>>& values,
      const layered::EndpointRange& endpoint_range = {}) {
    return integrateOptionalPointcloudLayer<
        LayerTagT,
        layered::PointcloudLayerIntegrationMode::kEndpointOnly>(
        pointcloud, values, endpoint_range);
  }

  template <typename LayerTagT,
            layered::PointcloudLayerIntegrationMode IntegrationMode>
  bool integrateOptionalPointcloudLayer(
      const PosedPointcloud<>& pointcloud,
      const std::vector<
          std::optional<layered::LayerValueT<LayerTagT>>>& values,
      const layered::EndpointRange& endpoint_range = {}) {
    return integratePointcloudLayerImpl<LayerTagT, IntegrationMode>(
        pointcloud, values.size(),
        [&values](size_t index) { return values[index]; }, endpoint_range);
  }

  template <typename LayerTagT>
  layered::LayerIntegrationResult integrateObservations(
      const std::vector<layered::LayerObservation<
          layered::LayerValueT<LayerTagT>>>& observations) {
    std::scoped_lock lock(mutex_);
    return layered_pipeline_.template integrate<LayerTagT>(observations);
  }

  template <typename LayerTagT>
  layered::LayerIntegrationResult integrateEndpointObservations(
      const std::vector<layered::LayerObservation<
          layered::LayerValueT<LayerTagT>>>& observations) {
    return integrateObservations<LayerTagT>(observations);
  }

  template <typename ObservationBatchT>
  layered::LayerIntegrationResult integrateObservationBatch(
      const ObservationBatchT& batch) {
    std::scoped_lock lock(mutex_);
    return layered_pipeline_.integrateBatch(batch);
  }

  bool saveLayeredMap(const std::filesystem::path& file_path) override {
    std::scoped_lock lock(mutex_);
    // Match the original save_map behavior: persist a fully thresholded
    // snapshot, including every typed continuous layer.
    layered_map_.continuousMap().threshold();
    return LayeredMapIoT::save(file_path, layered_map_);
  }

  bool loadLayeredMap(const std::filesystem::path& file_path,
                      std::string* error_message = nullptr) override {
    {
      std::scoped_lock lock(mutex_);
      // Load into the existing object so the core Pipeline keeps owning the
      // exact same continuous octree.
      if (!LayeredMapIoT::load(file_path, layered_map_, error_message)) {
        return false;
      }
    }
    return publishLayeredMap();
  }

  bool publishLayeredMap(const ros::Time& stamp = ros::Time::now()) override {
    wavemap_msgs::LayeredMap msg;
    {
      std::scoped_lock lock(mutex_);
      if (!convert::layeredMapToRosMsg<LayeredMapT, RosConverterT>(
              layered_map_, world_frame_, stamp, msg)) {
        return false;
      }
      convert::clearDiscreteLayerBundleDirtyState(
          layered_map_.discreteLayers());
    }

    layered_map_pub_.publish(msg);
    return true;
  }

 private:
  template <typename LayerTagT,
            layered::PointcloudLayerIntegrationMode IntegrationMode,
            typename ValueAccessorT>
  bool integratePointcloudLayerImpl(
      const PosedPointcloud<>& pointcloud, size_t value_count,
      ValueAccessorT&& value_at,
      const layered::EndpointRange& endpoint_range) {
    if (value_count != pointcloud.size()) {
      return false;
    }
    if constexpr (!layered::kIsWaveletCompatibleLayer<LayerTagT>) {
      static_assert(
          IntegrationMode ==
              layered::PointcloudLayerIntegrationMode::kEndpointOnly,
          "Discrete point fields can only describe occupied endpoints. Use "
          "positioned observations for other integration semantics.");
      using Value = layered::LayerValueT<LayerTagT>;
      std::vector<layered::LayerObservation<Value>> observations;
      observations.reserve(pointcloud.size());
      for (size_t point_index = 0u; point_index < pointcloud.size();
           ++point_index) {
        const auto value = value_at(point_index);
        if (!value) {
          continue;
        }
        const Point3D C_endpoint = pointcloud[point_index];
        if (!endpoint_range.contains(C_endpoint)) {
          continue;
        }
        observations.emplace_back(pointcloud.getPose() * C_endpoint, *value);
      }
      std::scoped_lock lock(mutex_);
      const auto result =
          layered_pipeline_.template integrate<LayerTagT>(observations);
      return result.rejected == 0u;
    } else {
      constexpr auto descriptor = layered::detail::descriptorForLayer<
          LayerTagT, typename LayeredMapT::ContinuousLayers>();
      const FloatingPoint cell_width =
          layered_map_.continuousMap().getMinCellWidth();
      const Point3D& W_origin = pointcloud.getOrigin();
      std::scoped_lock lock(mutex_);
      for (size_t point_index = 0u; point_index < pointcloud.size();
           ++point_index) {
        const auto value = value_at(point_index);
        if (!value) {
          continue;
        }
        const Point3D C_endpoint = pointcloud[point_index];
        if (!endpoint_range.contains(C_endpoint)) {
          continue;
        }
        const Point3D W_endpoint = pointcloud.getPose() * C_endpoint;
        const layered::LayerObservation<layered::LayerValueT<LayerTagT>>
            observation(W_endpoint, *value);
        layered::forEachPointcloudIntegrationIndex<IntegrationMode>(
            W_origin, W_endpoint, cell_width,
            [&](const Index3D& voxel_index) {
              layered::updateContinuousLayersAtVoxel(
                  layered_map_.continuousMap(), voxel_index,
                  [&](auto& layers) {
                    auto& current = descriptor.value(layers);
                    current = layered::LayerUpdatePolicyT<LayerTagT>{}(
                        current, observation);
                  });
            });
      }
      return true;
    }
  }

  void layeredUpdateCallback(
      const wavemap_msgs::LayeredMapUpdate::ConstPtr& msg) {
    if (msg->header.frame_id != world_frame_) {
      ROS_WARN_STREAM("Ignoring layered map update in frame '"
                      << msg->header.frame_id << "'; expected '"
                      << world_frame_ << "'.");
      return;
    }

    std::string error_message;
    {
      std::scoped_lock lock(mutex_);
      if (!UpdateConverterT::integrate(
              *msg, layered_pipeline_, &error_message)) {
        ROS_WARN_STREAM("Ignoring invalid layered map update: "
                        << error_message);
        return;
      }
    }
    // The periodic layered publish operation consumes the continuous block
    // timestamps and discrete dirty-parent state in one coherent update.
  }

  void advertiseServices() {
    save_layered_map_srv_ =
        nh_private_.advertiseService<wavemap_msgs::FilePath::Request,
                                     wavemap_msgs::FilePath::Response>(
            "save_layered_map", [this](auto& request, auto& response) {
              response.success = saveLayeredMap(request.file_path);
              return true;
            });

    load_layered_map_srv_ =
        nh_private_.advertiseService<wavemap_msgs::FilePath::Request,
                                     wavemap_msgs::FilePath::Response>(
            "load_layered_map", [this](auto& request, auto& response) {
              std::string error_message;
              response.success = loadLayeredMap(request.file_path, &error_message);
              if (!response.success && !error_message.empty()) {
                ROS_ERROR_STREAM(error_message);
              }
              return true;
            });

    request_full_layered_map_srv_ =
        nh_private_.advertiseService<std_srvs::Empty::Request,
                                     std_srvs::Empty::Response>(
            "layered_map_request_full", [this](auto&, auto&) {
              return publishLayeredMap();
            });
  }

  static constexpr int kQueueSize = 1;
  static constexpr bool kLatchPublisher = true;

  ros::NodeHandle nh_private_;
  std::string world_frame_;

  mutable std::mutex mutex_;
  LayeredMapT layered_map_;
  layered::LayeredPipeline<LayeredMapT> layered_pipeline_;

  ros::Publisher layered_map_pub_;
  ros::Subscriber layered_update_sub_;
  ros::ServiceServer load_layered_map_srv_;
  ros::ServiceServer save_layered_map_srv_;
  ros::ServiceServer request_full_layered_map_srv_;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_LAYERED_ROS_SERVER_EXTENSION_H_
