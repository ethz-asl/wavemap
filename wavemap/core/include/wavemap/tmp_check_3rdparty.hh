#pragma once

#define MY_MACRO 123

#include <glog/logging.h>

#include <Eigen/Dense>
#include <boost/preprocessor/repetition/enum.hpp>

namespace wavemap::core {
int add_int(int, int);
float add_float(float, float);
void check_3rdparty();
}  // namespace wavemap::core