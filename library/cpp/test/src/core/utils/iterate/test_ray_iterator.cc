#include <gtest/gtest.h>

#include "wavemap/core/common.h"
#include "wavemap/core/utils/iterate/ray_iterator.h"
#include "wavemap/core/utils/print/eigen.h"
#include "wavemap/test/fixture_base.h"
#include "wavemap/test/geometry_generator.h"

namespace wavemap {
template <typename TypeParamT>
class RayIteratorTest : public FixtureBase, public GeometryGenerator {};

template <int dim>
struct TypeParamTemplate {
  static constexpr int kDim = dim;
};
using TypeParams = ::testing::Types<TypeParamTemplate<2>, TypeParamTemplate<3>>;
TYPED_TEST_SUITE(RayIteratorTest, TypeParams, );

TYPED_TEST(RayIteratorTest, IterationOrderAndCompleteness) {
  constexpr int kDim = TypeParam::kDim;
  constexpr int kNumTestRays = 100;
  struct TestRay {
    Point<kDim> origin;
    Vector<kDim> translation;
  };

  // Create zero length, perfectly horizontal/vertical, and random test rays
  std::vector<TestRay> test_rays;
  test_rays.reserve(kNumTestRays);
  // Generate rays along all axes to test for degenerate cases
  const auto origin = Point<kDim>::Zero();
  test_rays.push_back({origin, origin});
  for (int dim_idx = 0; dim_idx < kDim; ++dim_idx) {
    const auto vector = TestFixture::getRandomSignedDistance(1e-3, 4e1) *
                        Point<kDim>::Unit(dim_idx);
    test_rays.push_back({origin, vector});
    test_rays.push_back({origin, -vector});
    test_rays.push_back({vector, origin});
    test_rays.push_back({-vector, origin});
  }
  // Fill the rest of the vector with random rays
  std::generate_n(test_rays.end(), kNumTestRays - test_rays.size(), [this]() {
    return TestRay{GeometryGenerator::getRandomPoint<kDim>(),
                   GeometryGenerator::getRandomTranslation<kDim>()};
  });

  for (const auto& test_ray : test_rays) {
    const Point<kDim> start_point = test_ray.origin;
    const Point<kDim> end_point = test_ray.origin + test_ray.translation;
    const Vector<kDim> t_start_end = end_point - start_point;
    const FloatingPoint ray_length = t_start_end.norm();

    const FloatingPoint min_cell_width = TestFixture::getRandomMinCellWidth();
    const FloatingPoint min_cell_width_inv = 1.f / min_cell_width;
    const Index<kDim> start_point_index =
        convert::pointToNearestIndex(start_point, min_cell_width_inv);
    const Index<kDim> end_point_index =
        convert::pointToNearestIndex(end_point, min_cell_width_inv);
    const Index<kDim> direction = (end_point_index - start_point_index)
                                      .cwiseSign()
                                      .template cast<IndexElement>();

    Ray ray(start_point, end_point, min_cell_width);
    EXPECT_EQ(*ray.begin(), start_point_index);
    EXPECT_EQ(*ray.end(), end_point_index);

    Index<kDim> last_index;
    size_t step_idx = 0u;
    for (const Index<kDim>& index : ray) {
      if (step_idx == 0u) {
        EXPECT_EQ(index, start_point_index)
            << "Ray iterator did not start at start index.";
      } else {
        EXPECT_NE(index, last_index)
            << "Ray iterator updated the same index twice.";
        const Index<kDim> index_diff = index - last_index;
        EXPECT_EQ(index_diff.cwiseAbs().sum(), 1)
            << "Ray iterator skipped a cell.";
        EXPECT_TRUE((index_diff.array() == direction.array()).any())
            << "Ray iterator stepped into an unexpected direction.";

        const Point<kDim> current_point =
            convert::indexToCenterPoint(index, min_cell_width);
        const Vector<kDim> t_start_current = current_point - start_point;
        const FloatingPoint distance =
            std::abs(t_start_end.x() * t_start_current.y() -
                     t_start_current.x() * t_start_end.y()) /
            ray_length;
        constexpr FloatingPoint kMaxAcceptableMultiplicativeError = 1.f + 1e-4f;
        EXPECT_LE(distance, kMaxAcceptableMultiplicativeError *
                                std::sqrt(0.5f) * min_cell_width)
            << "Ray iterator updated cell that is not traversed by the ray, "
               "for cell with index"
            << print::eigen::oneLine(index) << " and center point"
            << print::eigen::oneLine(current_point) << " on ray from"
            << print::eigen::oneLine(start_point) << " to"
            << print::eigen::oneLine(end_point) << " at min cell width "
            << min_cell_width << ".";
      }
      last_index = index;
      ++step_idx;
    }
    EXPECT_TRUE(last_index == end_point_index);
    EXPECT_GE(step_idx, 1)
        << "Ray iterator should at least take one step, to update the cell "
           "that contains the start (and end) point.";
  }
}
}  // namespace wavemap
