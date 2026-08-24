#include <sstream>
#include <string_view>
#include <type_traits>

#include <gtest/gtest.h>

#include <wavemap/layered/layer_schema.h>
#include <wavemap/layered/layer_observation_batch.h>
#include <wavemap/layered/layer_update_policy.h>
#include <wavemap/layered/layered_pipeline.h>
#include <wavemap/layered/layered_map_config_builder.h>
#include <wavemap/layered/layered_map_definition.h>

namespace wavemap::layered {
namespace {

struct ReflectivityLayer
    : schema::ContinuousLayer<float, ReplaceLayerUpdatePolicy<float>> {
  static constexpr std::string_view name = "reflectivity";
};

struct TemperatureLayer
    : schema::ContinuousLayer<float, ReplaceLayerUpdatePolicy<float>> {
  static constexpr std::string_view name = "temperature";
};

struct ClassLayer
    : schema::DiscreteLayer<int, ReplaceLayerUpdatePolicy<int>> {
  static constexpr std::string_view name = "class";
};

struct AveragedLayer
    : schema::StatefulContinuousLayer<float, WeightedMeanState,
                                      WeightedMeanLayerUpdatePolicy> {
  static constexpr std::string_view name = "averaged";
};

struct CustomDampedPolicy {
  float operator()(float current,
                   const LayerObservation<float>& observation) const {
    return 0.75f * current + 0.25f * observation.value;
  }
};

struct UnsupportedContinuousValue {
  float value = 0.f;
};

static_assert(kIsContinuousLayerUpdatePolicy<CustomDampedPolicy, float>);
static_assert(kIsDiscreteLayerUpdatePolicy<ReplaceLayerUpdatePolicy<int>, int>);
static_assert(schema::kHasContinuousValueTraits<float>);
static_assert(schema::kHasContinuousValueTraits<Rgb>);
static_assert(kHasLayerStreamCodec<float>);
static_assert(kHasLayerStreamCodec<Rgb>);
static_assert(!schema::kHasContinuousValueTraits<UnsupportedContinuousValue>);
static_assert(!kHasLayerStreamCodec<UnsupportedContinuousValue>);

using ContinuousDefinition =
    LayeredMapDefinition<schema::LayerSchema<ReflectivityLayer,
                                             TemperatureLayer>>;
using MixedDefinition =
    LayeredMapDefinition<schema::LayerSchema<ReflectivityLayer, ClassLayer>>;
using DiscreteDefinition =
    LayeredMapDefinition<schema::LayerSchema<ClassLayer>>;
using EmptyDefinition = LayeredMapDefinition<schema::LayerSchema<>>;
using WeightedDefinition =
    LayeredMapDefinition<schema::LayerSchema<AveragedLayer>>;

static_assert(std::is_same_v<typename ContinuousDefinition::Map::ContinuousLayers,
                             ContinuousDefinition::ContinuousLayers>);
static_assert(std::is_same_v<typename MixedDefinition::Map::DiscreteLayers,
                             MixedDefinition::DiscreteLayers>);
static_assert(ContinuousDefinition::Schema::continuousLayerCount == 2u);
static_assert(MixedDefinition::Schema::continuousLayerCount == 1u);
static_assert(MixedDefinition::Schema::discreteLayerCount == 1u);
static_assert(DiscreteDefinition::Schema::continuousLayerCount == 0u);
static_assert(EmptyDefinition::Schema::size == 0u);
static_assert(
    std::is_same_v<LayerValueT<AveragedLayer>, FloatingPoint>);
static_assert(
    std::is_same_v<LayerStateT<AveragedLayer>, WeightedMeanState>);
static_assert(schema::kHasContinuousValueTraits<WeightedMeanState>);
static_assert(kHasLayerStreamCodec<WeightedMeanState>);

template <typename DefinitionT>
typename DefinitionT::Config makeConfig() {
  typename DefinitionT::Config config;
  config.continuous_map.min_cell_width = 0.25f;
  config.continuous_map.min_log_odds = -2.f;
  config.continuous_map.max_log_odds = 4.f;
  config.continuous_map.tree_height = 4;
  return config;
}

TEST(LayeredMapDefinition, GeneratesContinuousStorage) {
  ContinuousDefinition::Map map(makeConfig<ContinuousDefinition>());
  const Index3D index{1, 2, 3};

  ContinuousDefinition::Voxel voxel;
  voxel.occupancy = 0.7f;
  voxel.data.get<ReflectivityLayer>() = 0.25f;
  voxel.data.get<TemperatureLayer>() = 18.f;
  map.continuousMap().setVoxelValue(index, voxel);

  const auto stored = map.continuousMap().getVoxelValue(index);
  EXPECT_FLOAT_EQ(stored.occupancy, 0.7f);
  EXPECT_FLOAT_EQ(stored.data.get<ReflectivityLayer>(), 0.25f);
  EXPECT_FLOAT_EQ(stored.data.get<TemperatureLayer>(), 18.f);
}

TEST(LayeredMapDefinition, GeneratesMixedContinuousAndDiscreteStorage) {
  MixedDefinition::Map map(makeConfig<MixedDefinition>());
  const Index3D index{-2, 1, 4};

  MixedDefinition::Voxel voxel;
  voxel.occupancy = 0.4f;
  voxel.data.get<ReflectivityLayer>() = 0.8f;
  map.continuousMap().setVoxelValue(index, voxel);
  map.discreteLayers().get<ClassLayer>().setValue(index, 3);

  EXPECT_FLOAT_EQ(map.continuousMap()
                      .getVoxelValue(index)
                      .data.get<ReflectivityLayer>(),
                  0.8f);
  ASSERT_TRUE(map.discreteLayers().get<ClassLayer>().getValue(index));
  EXPECT_EQ(*map.discreteLayers().get<ClassLayer>().getValue(index), 3);
}

TEST(LayeredMapDefinition, IntegratesHeterogeneousObservationBatch) {
  MixedDefinition::Map map(makeConfig<MixedDefinition>());
  LayeredPipeline<MixedDefinition::Map> pipeline(map);
  LayerObservationBatch<MixedDefinition::Schema> batch;
  const Point3D position(0.5f, 0.25f, -0.25f);

  batch.add<ReflectivityLayer>(position, 0.65f);
  batch.add<ClassLayer>(position, 4);
  EXPECT_EQ(batch.size(), 2u);

  const auto result = pipeline.integrateBatch(batch);
  EXPECT_EQ(result.received, 2u);
  EXPECT_EQ(result.integrated, 2u);
  EXPECT_EQ(result.rejected, 0u);

  const Index3D index = convert::pointToNearestIndex(
      position, 1.f / map.continuousMap().getMinCellWidth());
  EXPECT_FLOAT_EQ(map.continuousMap()
                      .getVoxelValue(index)
                      .data.get<ReflectivityLayer>(),
                  0.65f);
  ASSERT_TRUE(map.discreteLayers().get<ClassLayer>().getValue(index));
  EXPECT_EQ(*map.discreteLayers().get<ClassLayer>().getValue(index), 4);
}

TEST(LayeredMapDefinition, TransformsAllObservationBatchPositions) {
  LayerObservationBatch<MixedDefinition::Schema> batch;
  batch.add<ReflectivityLayer>(Point3D(1.f, 2.f, 3.f), 0.5f);
  batch.add<ClassLayer>(Point3D(-1.f, 0.f, 1.f), 2);

  const Transformation3D transform(Rotation3D(), Point3D(2.f, -1.f, 0.5f));
  batch.transformPositions(transform);

  EXPECT_TRUE(batch.observations<ReflectivityLayer>()[0].position.isApprox(
      Point3D(3.f, 1.f, 3.5f)));
  EXPECT_TRUE(batch.observations<ClassLayer>()[0].position.isApprox(
      Point3D(1.f, -1.f, 1.5f)));
}

TEST(LayeredMapDefinition, StoresTrueWeightedMeanFusionState) {
  WeightedDefinition::Map map(makeConfig<WeightedDefinition>());
  LayeredPipeline<WeightedDefinition::Map> pipeline(map);
  const Point3D position(0.5f, 0.25f, -0.25f);

  LayerObservation<float> first(position, 0.2f);
  first.confidence = 1.f;
  LayerObservation<float> second(position, 0.8f);
  second.confidence = 3.f;
  const std::vector<LayerObservation<float>> observations{first, second};
  const auto result = pipeline.integrate<AveragedLayer>(observations);

  EXPECT_EQ(result.integrated, 2u);
  EXPECT_EQ(result.updated_voxels, 1u);
  const Index3D index = convert::pointToNearestIndex(
      position, 1.f / map.continuousMap().getMinCellWidth());
  const auto state =
      map.continuousMap().getVoxelValue(index).data.get<AveragedLayer>();
  EXPECT_FLOAT_EQ(state.weighted_sum, 2.6f);
  EXPECT_FLOAT_EQ(state.total_weight, 4.f);
  EXPECT_FLOAT_EQ(LayerStateConversion<AveragedLayer>::value(state), 0.65f);
}

TEST(LayeredMapDefinition, WeightedMeanHasExplicitUnobservedState) {
  const WeightedMeanState unobserved;
  EXPECT_FLOAT_EQ(unobserved.total_weight, 0.f);
  EXPECT_FLOAT_EQ(unobserved.valueOr(-1.f), -1.f);

  LayerObservation<float> observation(Point3D::Zero(), 0.8f);
  const auto initialized =
      WeightedMeanLayerUpdatePolicy{}(unobserved, observation);
  EXPECT_FLOAT_EQ(initialized.weighted_sum, 0.8f);
  EXPECT_FLOAT_EQ(initialized.total_weight, 1.f);
  EXPECT_FLOAT_EQ(initialized.valueOr(), 0.8f);
}

TEST(LayeredMapDefinition, WeightedMeanFusionStateStreamRoundTrip) {
  const WeightedMeanState expected{2.6f, 4.f};
  std::stringstream stream;
  LayerStreamCodec<WeightedMeanState>::write(stream, expected);
  const auto restored = LayerStreamCodec<WeightedMeanState>::read(stream);
  EXPECT_FLOAT_EQ(restored.weighted_sum, expected.weighted_sum);
  EXPECT_FLOAT_EQ(restored.total_weight, expected.total_weight);
  EXPECT_FLOAT_EQ(restored.valueOr(), 0.65f);
}

TEST(LayeredMapDefinition, SupportsDiscreteOnlyAndEmptySchemas) {
  DiscreteDefinition::Map discrete_map(makeConfig<DiscreteDefinition>());
  EmptyDefinition::Map empty_map(makeConfig<EmptyDefinition>());
  const Index3D index{0, 0, 0};

  discrete_map.discreteLayers().get<ClassLayer>().setValue(index, 7);
  discrete_map.continuousMap().setCellValue(index, 0.5f);
  empty_map.continuousMap().setCellValue(index, 0.6f);

  EXPECT_EQ(*discrete_map.discreteLayers().get<ClassLayer>().getValue(index),
            7);
  EXPECT_FLOAT_EQ(discrete_map.continuousMap().getCellValue(index), 0.5f);
  EXPECT_FLOAT_EQ(empty_map.continuousMap().getCellValue(index), 0.6f);
}

TEST(LayeredMapDefinition, UnchangedDiscreteAssignmentDoesNotBecomeDirty) {
  DiscreteLayer<int> layer(DiscreteCompressionConfig{1});
  const Index3D index{2, 3, 4};

  layer.setValue(index, 7);
  ASSERT_EQ(layer.dirtyParentKeys().size(), 1u);
  layer.clearDirtyParentKeys();

  layer.setValue(index, 7);
  EXPECT_TRUE(layer.dirtyParentKeys().empty());

  layer.setValue(index, 8);
  EXPECT_EQ(layer.dirtyParentKeys().size(), 1u);
  EXPECT_EQ(layer.getValue(index), 8);
}

TEST(LayeredMapDefinition, BatchedDiscreteAssignmentMarksOnlyChangedParents) {
  DiscreteLayer<int> layer(DiscreteCompressionConfig{1});
  const Index3D unchanged{0, 0, 0};
  const Index3D changed{8, 0, 0};

  layer.setValues({{unchanged, 1}, {changed, 2}});
  layer.clearDirtyParentKeys();
  layer.setValues({{unchanged, 1}, {changed, 3}});

  ASSERT_EQ(layer.dirtyParentKeys().size(), 1u);
  EXPECT_EQ(layer.getValue(unchanged), 1);
  EXPECT_EQ(layer.getValue(changed), 3);

  layer.clearDirtyParentKeys();
  layer.setValues({{unchanged, 1}, {changed, 3}});
  EXPECT_TRUE(layer.dirtyParentKeys().empty());
}

TEST(LayeredMapDefinition, ConfigBuilderPopulatesExistingConfig) {
  LayeredMapConfigBuilder<MixedDefinition> builder;
  builder.map()
      .minCellWidth(0.2f)
      .occupancyLogOdds(-3.f, 5.f)
      .treeHeight(7)
      .onlyPruneBlocksIfUnusedFor(8.f);
  builder.occupancy().pruningScale(0.002f).pruningWeight(2.f);
  builder.layer<ReflectivityLayer>()
      .storageBounds(0.f, 1.f)
      .pruningScale(0.01f)
      .pruningWeight(0.5f);
  builder.combinedPruningThreshold(1.5f).discreteBlockHeight(2);

  const auto config = builder.build();
  EXPECT_FLOAT_EQ(config.continuous_map.min_cell_width, 0.2f);
  EXPECT_FLOAT_EQ(config.continuous_map.min_log_odds, -3.f);
  EXPECT_FLOAT_EQ(config.continuous_map.max_log_odds, 5.f);
  EXPECT_EQ(config.continuous_map.tree_height, 7);
  EXPECT_FLOAT_EQ(config.continuous_map.only_prune_blocks_if_unused_for, 8.f);
  EXPECT_FLOAT_EQ(config.continuous_pruning.occupancy.scale, 0.002f);
  EXPECT_FLOAT_EQ(config.continuous_pruning.occupancy.weight, 2.f);

  const auto& reflectivity_bounds =
      config.continuous_threshold.data.get<ReflectivityLayer>();
  const auto& reflectivity_pruning =
      config.continuous_pruning.data.get<ReflectivityLayer>();
  EXPECT_FLOAT_EQ(reflectivity_bounds.min, 0.f);
  EXPECT_FLOAT_EQ(reflectivity_bounds.max, 1.f);
  EXPECT_FLOAT_EQ(reflectivity_pruning.scale, 0.01f);
  EXPECT_FLOAT_EQ(reflectivity_pruning.weight, 0.5f);
  EXPECT_FLOAT_EQ(config.continuous_pruning.combined_threshold, 1.5f);
  EXPECT_FLOAT_EQ(config.continuous_pruning.data.combined_threshold, 1.5f);
  EXPECT_EQ(config.discrete_compression.block_height, 2);
}

TEST(LayeredUpdatePolicy, ProvidesStandardAndCustomPolicies) {
  const LayerObservation<float> low(Point3D::Zero(), 0.2f);
  const LayerObservation<float> high(Point3D::Zero(), 0.8f);

  EXPECT_FLOAT_EQ(ReplaceLayerUpdatePolicy<float>{}(0.5f, high), 0.8f);
  EXPECT_FLOAT_EQ(MinimumLayerUpdatePolicy<float>{}(0.5f, low), 0.2f);
  EXPECT_FLOAT_EQ(MaximumLayerUpdatePolicy<float>{}(0.5f, high), 0.8f);
  EXPECT_FLOAT_EQ(AccumulateLayerUpdatePolicy<float>{}(0.5f, low), 0.7f);
  EXPECT_FLOAT_EQ(CustomDampedPolicy{}(0.4f, high), 0.5f);

  using ExponentialPolicy = ExponentialScalarLayerUpdatePolicy<1, 5>;
  static_assert(kIsContinuousLayerUpdatePolicy<ExponentialPolicy, float>);
  EXPECT_FLOAT_EQ(ExponentialPolicy::observationWeight(), 0.2f);
  EXPECT_FLOAT_EQ(ExponentialPolicy{}(0.5f, high), 0.56f);

  float bounded_value = 0.f;
  for (int update_idx = 0; update_idx < 100; ++update_idx) {
    const auto& observation = update_idx % 2 ? low : high;
    bounded_value = ExponentialPolicy{}(bounded_value, observation);
    EXPECT_GE(bounded_value, 0.f);
    EXPECT_LE(bounded_value, 1.f);
  }

  EXPECT_TRUE(LogicalOrLayerUpdatePolicy{}(
      false, LayerObservation<bool>(Point3D::Zero(), true)));
  EXPECT_FALSE(LogicalAndLayerUpdatePolicy{}(
      true, LayerObservation<bool>(Point3D::Zero(), false)));
}

TEST(LayeredUpdatePolicy, ConfidenceBlendUsesObservationConfidence) {
  struct FloatArithmetic {
    static float add(float lhs, float rhs) { return lhs + rhs; }
    static float scale(float value, float factor) { return value * factor; }
  };

  LayerObservation<float> observation(Point3D::Zero(), 1.f);
  observation.confidence = 0.25f;
  const ConfidenceBlendLayerUpdatePolicy<float, FloatArithmetic> policy;
  EXPECT_FLOAT_EQ(policy(0.f, observation), 0.25f);

  observation.confidence.reset();
  const ConfidenceBlendLayerUpdatePolicy<float, FloatArithmetic>
      default_policy(0.4f);
  EXPECT_FLOAT_EQ(default_policy(0.f, observation), 0.4f);
}

}  // namespace
}  // namespace wavemap::layered
