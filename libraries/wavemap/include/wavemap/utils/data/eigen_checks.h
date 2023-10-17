#ifndef WAVEMAP_UTILS_DATA_EIGEN_CHECKS_H_
#define WAVEMAP_UTILS_DATA_EIGEN_CHECKS_H_

#include <functional>

#include <glog/logging.h>

#include "wavemap/common.h"
#include "wavemap/utils/data/comparisons.h"
#include "wavemap/utils/print/eigen.h"

// Define regular checks
#define CHECK_EIGEN_EQ(MatrixA, MatrixB)                                     \
  CHECK(wavemap::data::EigenCwise<std::equal_to<>>(MatrixA, MatrixB))        \
      << "For matrix A" << print::eigen::oneLine(MatrixA) << " and matrix B" \
      << print::eigen::oneLine(MatrixB)

#define CHECK_EIGEN_NE(MatrixA, MatrixB)                                     \
  CHECK(wavemap::data::EigenCwise<std::not_equal_to<>>(MatrixA, MatrixB))    \
      << "For matrix A" << print::eigen::oneLine(MatrixA) << " and matrix B" \
      << print::eigen::oneLine(MatrixB)

#define CHECK_EIGEN_LE(MatrixA, MatrixB)                                     \
  CHECK(wavemap::data::EigenCwise<std::less_equal<>>(MatrixA, MatrixB))      \
      << "For matrix A" << print::eigen::oneLine(MatrixA) << " and matrix B" \
      << print::eigen::oneLine(MatrixB)

#define CHECK_EIGEN_LT(MatrixA, MatrixB)                                     \
  CHECK(wavemap::data::EigenCwise<std::less<>>(MatrixA, MatrixB))            \
      << "For matrix A" << print::eigen::oneLine(MatrixA) << " and matrix B" \
      << print::eigen::oneLine(MatrixB)

#define CHECK_EIGEN_GE(MatrixA, MatrixB)                                     \
  CHECK(wavemap::data::EigenCwise<std::greater_equal<>>(MatrixA, MatrixB))   \
      << "For matrix A" << print::eigen::oneLine(MatrixA) << " and matrix B" \
      << print::eigen::oneLine(MatrixB)

#define CHECK_EIGEN_GT(MatrixA, MatrixB)                                     \
  CHECK(wavemap::data::EigenCwise<std::greater<>>(MatrixA, MatrixB))         \
      << "For matrix A" << print::eigen::oneLine(MatrixA) << " and matrix B" \
      << print::eigen::oneLine(MatrixB)

#define CHECK_EIGEN_NEAR(MatrixA, MatrixB, Precision)                        \
  CHECK(wavemap::data::EigenCwiseNear(MatrixA, MatrixB, Precision))          \
      << "For matrix A" << print::eigen::oneLine(MatrixA) << " and matrix B" \
      << print::eigen::oneLine(MatrixB) << " with precision " << Precision

// Define debug checks
#if DCHECK_IS_ON()

#define DCHECK_EIGEN_EQ(MatrixA, MatrixB) CHECK_EIGEN_EQ(MatrixA, MatrixB)
#define DCHECK_EIGEN_NE(MatrixA, MatrixB) CHECK_EIGEN_NE(MatrixA, MatrixB)
#define DCHECK_EIGEN_LE(MatrixA, MatrixB) CHECK_EIGEN_LE(MatrixA, MatrixB)
#define DCHECK_EIGEN_LT(MatrixA, MatrixB) CHECK_EIGEN_LT(MatrixA, MatrixB)
#define DCHECK_EIGEN_GE(MatrixA, MatrixB) CHECK_EIGEN_GE(MatrixA, MatrixB)
#define DCHECK_EIGEN_GT(MatrixA, MatrixB) CHECK_EIGEN_GT(MatrixA, MatrixB)
#define DCHECK_EIGEN_NEAR(MatrixA, MatrixB, Precision) \
  CHECK_EIGEN_NEAR(MatrixA, MatrixB, Precision)

// When debug checking is disabled, replace the DCHECKs with empty macros
#else

#define DCHECK_EIGEN_EQ(MatrixA, MatrixB) \
  GLOG_MSVC_PUSH_DISABLE_WARNING(4127)    \
  while (false) GLOG_MSVC_POP_WARNING() CHECK_EIGEN_EQ(MatrixA, MatrixB)

#define DCHECK_EIGEN_NE(MatrixA, MatrixB) \
  GLOG_MSVC_PUSH_DISABLE_WARNING(4127)    \
  while (false) GLOG_MSVC_POP_WARNING() CHECK_EIGEN_NE(MatrixA, MatrixB)

#define DCHECK_EIGEN_LE(MatrixA, MatrixB) \
  GLOG_MSVC_PUSH_DISABLE_WARNING(4127)    \
  while (false) GLOG_MSVC_POP_WARNING() CHECK_EIGEN_LE(MatrixA, MatrixB)

#define DCHECK_EIGEN_LT(MatrixA, MatrixB) \
  GLOG_MSVC_PUSH_DISABLE_WARNING(4127)    \
  while (false) GLOG_MSVC_POP_WARNING() CHECK_EIGEN_LT(MatrixA, MatrixB)

#define DCHECK_EIGEN_GE(MatrixA, MatrixB) \
  GLOG_MSVC_PUSH_DISABLE_WARNING(4127)    \
  while (false) GLOG_MSVC_POP_WARNING() CHECK_EIGEN_GE(MatrixA, MatrixB)

#define DCHECK_EIGEN_GT(MatrixA, MatrixB) \
  GLOG_MSVC_PUSH_DISABLE_WARNING(4127)    \
  while (false) GLOG_MSVC_POP_WARNING() CHECK_EIGEN_GT(MatrixA, MatrixB)

#define DCHECK_EIGEN_NEAR(MatrixA, MatrixB, Precision) \
  GLOG_MSVC_PUSH_DISABLE_WARNING(4127)                 \
  while (false)                                        \
  GLOG_MSVC_POP_WARNING() CHECK_EIGEN_NEAR(MatrixA, MatrixB, Precision)

#endif

#endif  // WAVEMAP_UTILS_DATA_EIGEN_CHECKS_H_
