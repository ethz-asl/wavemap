#ifndef WAVEMAP_ROS_ROS_SERVER_H_
#define WAVEMAP_ROS_ROS_SERVER_H_

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <glog/logging.h>
#include <ros/ros.h>
#include <wavemap/core/common.h>
#include <wavemap/core/config/config_base.h>
#include <wavemap/core/indexing/index_hashes.h>
#include <wavemap/core/integrator/integrator_base.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/core/utils/thread_pool.h>
#include <wavemap/layered/classification/local_elevation_classifier.h>
#include <wavemap/pipeline/pipeline.h>

#include "wavemap_ros/inputs/ros_input_base.h"
#include "wavemap_ros/inputs/pointcloud_topic_input.h"
#include "wavemap_ros/inputs/sensor_frame_transformer.h"
#include "wavemap_ros/layered_ros_server_extension.h"
#include "wavemap_ros/map_operations/publish_map_operation.h"
#include "wavemap_ros/utils/ros_logging_level.h"
#include "wavemap_ros/utils/tf_transformer.h"

namespace wavemap {
template <typename MapDefinitionT, typename RosConverterT>
class LayeredRosServerBuilder;

/**
 * Config struct for wavemap's ROS server.
 */
struct RosServerConfig : ConfigBase<RosServerConfig, 4, RosLoggingLevel> {
  //! Name of the coordinate frame in which to store the map.
  //! Will be used as the frame_id for ROS TF lookups.
  std::string world_frame = "odom";
  //! Minimum severity level for messages to be logged.
  RosLoggingLevel logging_level = RosLoggingLevel::kInfo;
  //! Maximum number of threads to use.
  //! Defaults to the number of threads supported by the CPU.
  int num_threads =
      std::max(1, static_cast<int>(std::thread::hardware_concurrency()));
  //! Whether or not to allow resetting the map through the reset_map service.
  bool allow_reset_map_service = false;

  static MemberMap memberMap;

  bool isValid(bool verbose) const override;
};

class RosServer {
 public:
  using MapInstaller =
      std::function<void(RosServer&, ros::NodeHandle)>;

  RosServer(ros::NodeHandle nh, ros::NodeHandle nh_private);
  RosServer(ros::NodeHandle nh, ros::NodeHandle nh_private,
            MapInstaller map_installer);
  RosServer(ros::NodeHandle nh, ros::NodeHandle nh_private,
            const RosServerConfig& config);
  RosServer(ros::NodeHandle nh, ros::NodeHandle nh_private,
            const RosServerConfig& config, MapInstaller map_installer);

  void clear();

  MapBase::Ptr getMap() { return occupancy_map_; }
  MapBase::ConstPtr getMap() const { return occupancy_map_; }

  Pipeline& getPipeline() { return *pipeline_; }
  const Pipeline& getPipeline() const { return *pipeline_; }
  const std::string& worldFrame() const { return config_.world_frame; }

  SensorFrameTransformer makeSensorFrameTransformer() const {
    return SensorFrameTransformer(transformer_, config_.world_frame);
  }

  MapOperationBase* addOperation(const param::Value& operation_params,
                                 ros::NodeHandle nh_private);

  RosInputBase* addInput(const param::Value& integrator_params,
                         const ros::NodeHandle& nh, ros::NodeHandle nh_private);
  RosInputBase* addInput(std::unique_ptr<RosInputBase> input);
  using PointcloudEndpointAdapterRegistrar =
      std::function<void(PointcloudTopicInput&)>;
  void addPointcloudEndpointAdapterRegistrar(
      PointcloudEndpointAdapterRegistrar registrar);
  template <typename LayerTagT,
            layered::PointcloudLayerIntegrationMode IntegrationMode =
                layered::PointcloudLayerIntegrationMode::kEndpointOnly,
            typename LayeredExtensionT>
  void bindPointField(
      LayeredExtensionT& extension,
      PointFieldBinding<layered::LayerValueT<LayerTagT>> binding = {},
      layered::EndpointRange endpoint_range = {}) {
    using Value = layered::LayerValueT<LayerTagT>;
    static_assert(std::is_arithmetic_v<Value>,
                  "bindPointField supports numeric scalar layers. Use a "
                  "custom PointcloudEndpointAdapter for structured values.");
    if (binding.field_name.empty()) {
      binding.field_name =
          std::string(layered::LayerTraits<LayerTagT>::name);
    }
    addPointcloudEndpointAdapterRegistrar(
        [&extension, binding = std::move(binding), endpoint_range](
            PointcloudTopicInput& input) {
          using Adapter = NumericPointcloudEndpointAdapter<Value>;
          input.addEndpointAdapter(std::make_shared<Adapter>(
              binding.field_name, binding.scale, binding.offset,
              binding.min_value, binding.max_value,
              [&extension, endpoint_range](const PosedPointcloud<>& pointcloud,
                           const std::vector<Value>& values) {
                extension.template integratePointcloudLayer<LayerTagT,
                                                              IntegrationMode>(
                    pointcloud, values, endpoint_range);
              }));
        });
  }
  template <typename LayerTagT,
            layered::PointcloudLayerIntegrationMode IntegrationMode =
                layered::PointcloudLayerIntegrationMode::kEndpointOnly,
            typename LayeredExtensionT>
  void bindNormalizedPointField(
      LayeredExtensionT& extension, std::string field_name,
      FloatingPoint raw_min, FloatingPoint raw_max,
      layered::EndpointRange endpoint_range = {}) {
    using Value = layered::LayerValueT<LayerTagT>;
    static_assert(std::is_arithmetic_v<Value>,
                  "Normalized point fields require a numeric scalar layer.");
    bindPointField<LayerTagT, IntegrationMode>(
        extension,
        PointFieldBinding<Value>::normalized(std::move(field_name), raw_min,
                                             raw_max),
        endpoint_range);
  }
  template <typename LayerTagT,
            layered::PointcloudLayerIntegrationMode IntegrationMode =
                layered::PointcloudLayerIntegrationMode::kEndpointOnly,
            typename LayeredExtensionT>
  void bindDirectPointField(
      LayeredExtensionT& extension, std::string field_name = {},
      layered::EndpointRange endpoint_range = {}) {
    using Value = layered::LayerValueT<LayerTagT>;
    static_assert(std::is_arithmetic_v<Value>,
                  "Direct point fields require a numeric scalar layer.");
    bindPointField<LayerTagT, IntegrationMode>(
        extension, PointFieldBinding<Value>(std::move(field_name)),
        endpoint_range);
  }
  template <typename LayerTagT,
            layered::PointcloudLayerIntegrationMode IntegrationMode =
                layered::PointcloudLayerIntegrationMode::kEndpointOnly,
            typename LayeredExtensionT, typename ConverterT>
  void bindComputedPointField(
      LayeredExtensionT& extension,
      std::vector<std::string> source_field_names, ConverterT converter,
      layered::EndpointRange endpoint_range = {}) {
    using Value = layered::LayerValueT<LayerTagT>;
    using FieldValues = NumericPointFieldValues;
    using OptionalValue = std::optional<Value>;
    static_assert(
        std::is_invocable_r_v<OptionalValue, ConverterT,
                              const FieldValues&>,
        "A computed point-field converter must return optional<LayerValue> "
        "and accept const NumericPointFieldValues&.");
    addPointcloudEndpointAdapterRegistrar(
        [&extension, source_field_names = std::move(source_field_names),
         converter = std::move(converter), endpoint_range](
            PointcloudTopicInput& input) {
          using Adapter = ComputedNumericPointcloudEndpointAdapter<Value>;
          input.addEndpointAdapter(std::make_shared<Adapter>(
              source_field_names, converter,
              [&extension, endpoint_range](
                  const PosedPointcloud<>& pointcloud,
                  const std::vector<OptionalValue>& values) {
                extension.template integrateOptionalPointcloudLayer<
                    LayerTagT, IntegrationMode>(
                    pointcloud, values, endpoint_range);
              }));
        });
  }
  template <typename LayerTagT, typename LayeredExtensionT>
  void bindGeometricClass(
      LayeredExtensionT& extension,
      layered::LocalElevationClassifierConfig classifier_config = {},
      layered::EndpointRange endpoint_range = {},
      Radians<FloatingPoint> min_azimuth = -kPi,
      Radians<FloatingPoint> max_azimuth = kPi) {
    using Value = layered::LayerValueT<LayerTagT>;
    static_assert(std::is_integral_v<Value>,
                  "Geometric class layers require an integral value type.");
    addPointcloudEndpointAdapterRegistrar(
        [&extension, classifier = layered::LocalElevationClassifier(
                         classifier_config), endpoint_range, min_azimuth,
         max_azimuth](
            PointcloudTopicInput& input) {
          input.addPosedCloudCallback(
              [&extension, classifier, endpoint_range, min_azimuth,
               max_azimuth](const PosedPointcloud<>& pointcloud) {
                std::vector<Point3D> class_points;
                class_points.reserve(pointcloud.size());
                for (const Point3D& point : pointcloud) {
                  const FloatingPoint azimuth = std::atan2(point.y(), point.x());
                  if (min_azimuth <= azimuth && azimuth <= max_azimuth) {
                    class_points.emplace_back(point);
                  }
                }
                const PosedPointcloud<> class_pointcloud(pointcloud.getPose(),
                                                          class_points);
                const auto observations =
                    layered::makeGeometricClassObservations<Value>(
                        class_pointcloud, classifier, endpoint_range);
                const auto result =
                    extension.template integrateObservations<
                        LayerTagT>(observations);
                return result.rejected == 0u;
              });
        });
  }
  const std::vector<std::unique_ptr<RosInputBase>>& getInputs() {
    return inputs_;
  }
  void clearInputs() { inputs_.clear(); }

  bool saveMap(const std::filesystem::path& file_path) const;
  bool loadMap(const std::filesystem::path& file_path);

 private:
  template <typename MapDefinitionT, typename RosConverterT>
  friend class LayeredRosServerBuilder;

  const RosServerConfig config_;

  // Map data structure
  MapBase::Ptr occupancy_map_;
  std::unique_ptr<LayeredRosServerExtensionBase> layered_extension_;
  using LayeredIntegratorFactory = std::function<std::unique_ptr<IntegratorBase>(
      const param::Value&, std::shared_ptr<ThreadPool>)>;
  LayeredIntegratorFactory layered_integrator_factory_;
  using LayeredPublishOperationFactory =
      std::function<std::unique_ptr<MapOperationBase>(
          const PublishMapOperationConfig&, ros::NodeHandle)>;
  LayeredPublishOperationFactory layered_publish_operation_factory_;
  std::vector<PointcloudEndpointAdapterRegistrar>
      pointcloud_endpoint_adapter_registrars_;

  // Threadpool shared among all input handlers and operations
  std::shared_ptr<ThreadPool> thread_pool_;

  // Map management pipeline
  std::shared_ptr<Pipeline> pipeline_;
  param::Array map_operation_param_array_;// dt_architecture_changes
  param::Map measurement_integrator_param_map_;// dt_architecture_changes
  param::Array input_param_array_;// dt_architecture_changes

  // Measurement and pose inputs
  std::vector<std::unique_ptr<RosInputBase>> inputs_;
  std::shared_ptr<TfTransformer> transformer_;

  // ROS services
  void advertiseServices(ros::NodeHandle& nh_private);
  ros::ServiceServer reset_map_srv_;
  ros::ServiceServer save_map_srv_;
  ros::ServiceServer load_map_srv_;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_ROS_SERVER_H_
