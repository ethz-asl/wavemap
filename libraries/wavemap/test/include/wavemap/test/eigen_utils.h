#ifndef WAVEMAP_TEST_EIGEN_UTILS_H_
#define WAVEMAP_TEST_EIGEN_UTILS_H_

#include <functional>
#include <string>

#include <gtest/gtest.h>

#include "wavemap/utils/print/eigen.h"

namespace wavemap {
template <typename ComparisonOp, typename EigenA, typename EigenB>
::testing::AssertionResult EigenCwise(const std::string& A_str,
                                      const EigenA& A_matrix,
                                      const std::string& op_str,
                                      const std::string& B_str,
                                      const EigenB& B_matrix) {
  static_assert(static_cast<Eigen::Index>(EigenA::RowsAtCompileTime) ==
                static_cast<Eigen::Index>(EigenB::RowsAtCompileTime));
  static_assert(static_cast<Eigen::Index>(EigenA::ColsAtCompileTime) ==
                static_cast<Eigen::Index>(EigenB::ColsAtCompileTime));
  if (ComparisonOp{}(A_matrix.array(), B_matrix.array()).all()) {
    return ::testing::AssertionSuccess();
  } else {
    return ::testing::AssertionFailure()
           << "Expected " << A_str << print::eigen::oneLine(A_matrix) << " "
           << op_str << " " << B_str << print::eigen::oneLine(B_matrix);
  }
}

template <typename EigenA, typename EigenB>
::testing::AssertionResult EigenCwiseNear(const std::string& A_str,
                                          const EigenA& A_matrix,
                                          const std::string& B_str,
                                          const EigenB& B_matrix,
                                          const std::string& tolerance_str,
                                          FloatingPoint tolerance = kEpsilon) {
  static_assert(EigenA::RowsAtCompileTime == EigenB::RowsAtCompileTime);
  static_assert(EigenA::ColsAtCompileTime == EigenB::ColsAtCompileTime);
  if (((A_matrix - B_matrix).array().abs() < tolerance).all()) {
    return ::testing::AssertionSuccess();
  } else {
    return ::testing::AssertionFailure()
           << "Expected " << A_str << print::eigen::oneLine(A_matrix)
           << " near " << B_str << print::eigen::oneLine(B_matrix)
           << " with tolerance (" << tolerance_str << ") " << tolerance;
  }
}

#define EXPECT_EIGEN_EQ(MatrixA, MatrixB) \
  EXPECT_TRUE(                            \
      EigenCwise<std::equal_to<>>(#MatrixA, MatrixA, "==", #MatrixB, MatrixB))

#define EXPECT_EIGEN_LT(MatrixA, MatrixB) \
  EXPECT_TRUE(                            \
      EigenCwise<std::less<>>(#MatrixA, MatrixA, "<", #MatrixB, MatrixB))

#define EXPECT_EIGEN_LE(MatrixA, MatrixB)                                      \
  EXPECT_TRUE(EigenCwise<std::less_equal<>>(#MatrixA, MatrixA, "<=", #MatrixB, \
                                            MatrixB))

#define EXPECT_EIGEN_GT(MatrixA, MatrixB) \
  EXPECT_TRUE(                            \
      EigenCwise<std::greater<>>(#MatrixA, MatrixA, ">", #MatrixB, MatrixB))

#define EXPECT_EIGEN_GE(MatrixA, MatrixB)                         \
  EXPECT_TRUE(EigenCwise<std::greater_equal<>>(#MatrixA, MatrixA, \
                                               ">=", #MatrixB, MatrixB))

#define EXPECT_EIGEN_NEAR(MatrixA, MatrixB, Tolerance)                         \
  EXPECT_TRUE(EigenCwiseNear(#MatrixA, MatrixA, #MatrixB, MatrixB, #Tolerance, \
                             Tolerance))
}  // namespace wavemap

#endif  // WAVEMAP_TEST_EIGEN_UTILS_H_
