#include <gtest/gtest.h>

#include <wavemap/layered/classification/local_elevation_classifier.h>

namespace wavemap::layered {
namespace {

PosedPointcloud<> makePointcloud(std::vector<Point3D> points,
                                 const Transformation3D& pose = {}) {
  return PosedPointcloud<>(pose, Pointcloud<>(points));
}

TEST(LocalElevationClassifierTest, ValidatesDefaultConfig) {
  EXPECT_TRUE(LocalElevationClassifierConfig{}.isValid());

  LocalElevationClassifierConfig config;
  config.obstacle_height = config.ground_tolerance;
  EXPECT_FALSE(config.isValid());
}

TEST(LocalElevationClassifierTest, ClassifiesSupportedCellByRelativeHeight) {
  LocalElevationClassifierConfig config;
  config.min_points_per_cell = 3u;
  const LocalElevationClassifier classifier(config);
  const auto cloud = makePointcloud({Point3D(0.1f, 0.1f, 1.f),
                                     Point3D(0.2f, 0.1f, 1.1f),
                                     Point3D(0.3f, 0.1f, 1.3f),
                                     Point3D(0.4f, 0.1f, 1.5f)});

  const auto classes = classifier.classify(cloud);

  ASSERT_EQ(classes.size(), 4u);
  EXPECT_EQ(classes[0], GeometricClass::kGround);
  EXPECT_EQ(classes[1], GeometricClass::kGround);
  EXPECT_EQ(classes[2], GeometricClass::kUnknown);
  EXPECT_EQ(classes[3], GeometricClass::kObstacle);
}

TEST(LocalElevationClassifierTest, LeavesSparseCellsUnknown) {
  LocalElevationClassifierConfig config;
  config.min_points_per_cell = 3u;
  const LocalElevationClassifier classifier(config);
  const auto cloud = makePointcloud(
      {Point3D(-0.1f, -0.1f, 0.f), Point3D(-0.2f, -0.1f, 1.f)});

  const auto classes = classifier.classify(cloud);

  ASSERT_EQ(classes.size(), 2u);
  EXPECT_EQ(classes[0], GeometricClass::kUnknown);
  EXPECT_EQ(classes[1], GeometricClass::kUnknown);
}

TEST(LocalElevationClassifierTest, UsesWorldCoordinates) {
  LocalElevationClassifierConfig config;
  config.min_points_per_cell = 2u;
  const LocalElevationClassifier classifier(config);
  Transformation3D pose;
  pose.getPosition() = Point3D(2.f, 3.f, 4.f);
  const auto cloud = makePointcloud(
      {Point3D(0.1f, 0.1f, 0.f), Point3D(0.2f, 0.1f, 0.5f)}, pose);

  const auto classes = classifier.classify(cloud);

  ASSERT_EQ(classes.size(), 2u);
  EXPECT_EQ(classes[0], GeometricClass::kGround);
  EXPECT_EQ(classes[1], GeometricClass::kObstacle);
}

TEST(LocalElevationClassifierTest, CreatesOnlyObservedWorldFrameUpdates) {
  LocalElevationClassifierConfig config;
  config.min_points_per_cell = 3u;
  const LocalElevationClassifier classifier(config);
  Transformation3D pose;
  pose.getPosition() = Point3D(2.f, 3.f, 4.f);
  const auto cloud = makePointcloud({Point3D(0.1f, 0.1f, 0.f),
                                     Point3D(0.2f, 0.1f, 0.3f),
                                     Point3D(0.3f, 0.1f, 0.5f)},
                                    pose);

  const auto observations =
      makeGeometricClassObservations<int>(cloud, classifier);

  ASSERT_EQ(observations.size(), 2u);
  EXPECT_EQ(observations[0].value,
            static_cast<int>(GeometricClass::kGround));
  EXPECT_EQ(observations[1].value,
            static_cast<int>(GeometricClass::kObstacle));
  EXPECT_TRUE(observations[0].position.isApprox(Point3D(2.1f, 3.1f, 4.f)));
  EXPECT_TRUE(observations[1].position.isApprox(Point3D(2.3f, 3.1f, 4.5f)));
}

TEST(LocalElevationClassifierTest, FiltersObservationsBySensorRange) {
  LocalElevationClassifierConfig config;
  config.grid_cell_width = 10.f;
  config.min_points_per_cell = 3u;
  const LocalElevationClassifier classifier(config);
  const auto cloud = makePointcloud({Point3D(0.1f, 0.f, 0.f),
                                     Point3D(1.f, 0.f, 0.f),
                                     Point3D(2.f, 0.f, 0.5f)});

  const auto observations = makeGeometricClassObservations<int>(
      cloud, classifier, EndpointRange{0.5f, 1.5f});

  ASSERT_EQ(observations.size(), 1u);
  EXPECT_TRUE(observations.front().position.isApprox(Point3D(1.f, 0.f, 0.f)));
}

}  // namespace
}  // namespace wavemap::layered
