#ifndef WAVEMAP_COMMON_TEST_EIGEN_UTILS_H_
#define WAVEMAP_COMMON_TEST_EIGEN_UTILS_H_

#include <functional>

#include <gtest/gtest.h>

#include "wavemap_common/utils/eigen_format.h"

namespace wavemap {
template <typename ComparisonOp, typename EigenA, typename EigenB>
::testing::AssertionResult EigenCwise(const EigenA& matrix_a,
                                      const EigenB& matrix_b) {
  static_assert(static_cast<Eigen::Index>(EigenA::RowsAtCompileTime) ==
                static_cast<Eigen::Index>(EigenB::RowsAtCompileTime));
  static_assert(static_cast<Eigen::Index>(EigenA::ColsAtCompileTime) ==
                static_cast<Eigen::Index>(EigenB::ColsAtCompileTime));
  if (ComparisonOp{}(matrix_a.array(), matrix_b.array()).all()) {
    return ::testing::AssertionSuccess();
  } else {
    return ::testing::AssertionFailure()
           << "for matrix A" << EigenFormat::oneLine(matrix_a)
           << " and matrix B" << EigenFormat::oneLine(matrix_b);
  }
}

template <typename EigenA, typename EigenB>
::testing::AssertionResult EigenCwiseNear(const EigenA& matrix_a,
                                          const EigenB& matrix_b,
                                          FloatingPoint precision = kEpsilon) {
  static_assert(EigenA::RowsAtCompileTime == EigenB::RowsAtCompileTime);
  static_assert(EigenA::ColsAtCompileTime == EigenB::ColsAtCompileTime);
  if (((matrix_a - matrix_b).array().abs() < precision).all()) {
    return ::testing::AssertionSuccess();
  } else {
    return ::testing::AssertionFailure()
           << "for matrix A" << EigenFormat::oneLine(matrix_a)
           << " and matrix B" << EigenFormat::oneLine(matrix_b)
           << " with precision " << precision;
  }
}

#define EXPECT_EIGEN_EQ(MatrixA, MatrixB) \
  EXPECT_TRUE(EigenCwise<std::equal_to<>>(MatrixA, MatrixB))

#define EXPECT_EIGEN_LT(MatrixA, MatrixB) \
  EXPECT_TRUE(EigenCwise<std::less<>>(MatrixA, MatrixB))

#define EXPECT_EIGEN_LE(MatrixA, MatrixB) \
  EXPECT_TRUE(EigenCwise<std::less_equal<>>(MatrixA, MatrixB))

#define EXPECT_EIGEN_GT(MatrixA, MatrixB) \
  EXPECT_TRUE(EigenCwise<std::greater<>>(MatrixA, MatrixB))

#define EXPECT_EIGEN_GE(MatrixA, MatrixB) \
  EXPECT_TRUE(EigenCwise<std::greater_equal<>>(MatrixA, MatrixB))

#define EXPECT_EIGEN_NEAR(MatrixA, MatrixB, Precision) \
  EXPECT_TRUE(EigenCwiseNear(MatrixA, MatrixB, Precision))
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_TEST_EIGEN_UTILS_H_
