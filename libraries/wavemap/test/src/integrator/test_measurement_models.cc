#include <gtest/gtest.h>

#include "wavemap/common.h"
#include "wavemap/integrator/measurement_model/constant_ray.h"
#include "wavemap/integrator/measurement_model/continuous_beam.h"
#include "wavemap/integrator/measurement_model/continuous_ray.h"
#include "wavemap/test/fixture_base.h"
#include "wavemap/utils/iterate/grid_iterator.h"
#include "wavemap/utils/iterate/ray_iterator.h"

namespace wavemap {
using MeasurementModelTest = FixtureBase;

TEST_F(MeasurementModelTest, DISABLED_ConstantRay) {
  // TODO(victorr): Implement this test
  constexpr int kNumRepetitions = 100;
  for (int i = 0; i < kNumRepetitions; ++i) {
  }
}

TEST_F(MeasurementModelTest, DISABLED_ContinuousRay) {
  // TODO(victorr): Implement this test
  constexpr int kNumRepetitions = 100;
  for (int i = 0; i < kNumRepetitions; ++i) {
  }
}

TEST_F(MeasurementModelTest, DISABLED_ContinuousBeam) {
  // TODO(victorr): Implement this test
  constexpr int kNumRepetitions = 100;
  for (int i = 0; i < kNumRepetitions; ++i) {
  }
}
}  // namespace wavemap
