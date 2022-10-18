#include <gtest/gtest.h>

#include "wavemap_common/common.h"
#include "wavemap_common/integrator/projection_model/image_2d/spherical_projector.h"
#include "wavemap_common/test/eigen_utils.h"
#include "wavemap_common/test/fixture_base.h"
#include "wavemap_common/utils/eigen_format.h"

namespace wavemap {
using SphericalProjectorTest = FixtureBase;

TEST_F(SphericalProjectorTest, InitializationAndAccessors) {
  for (int repetition = 0; repetition < 1000; ++repetition) {
    const FloatingPoint min_elevation_angle =
        getRandomAngle(-kHalfPi, kQuarterPi);
    const FloatingPoint max_elevation_angle =
        getRandomAngle(min_elevation_angle + kEpsilon, kHalfPi);
    const IndexElement num_rows = getRandomIndexElement(2, 1000);

    const FloatingPoint min_azimuth_angle = getRandomAngle(-kPi, kHalfPi);
    const FloatingPoint max_azimuth_angle =
        getRandomAngle(min_azimuth_angle + kEpsilon, kPi);
    const IndexElement num_columns = getRandomIndexElement(2, 1000);

    const Index2D dimensions{num_rows, num_columns};

    SphericalProjector spherical_projector(
        {{min_elevation_angle, max_elevation_angle, num_rows},
         {min_azimuth_angle, max_azimuth_angle, num_columns}});
    EXPECT_EQ(spherical_projector.getNumRows(), num_rows);
    EXPECT_EQ(spherical_projector.getNumColumns(), num_columns);
    EXPECT_EQ(spherical_projector.getDimensions(), dimensions);
  }
}

// TODO(victorr): Test boundaries
}  // namespace wavemap
