#include <iostream>
#include <tracy/Tracy.hpp>

#include "wavemap/tmp_check_3rdparty.hh"
#define PRINT_NUMBER(z, n, data) std::cout << n << " ";

namespace wavemap::core {
int add_int(int a, int b) {
  return a + b;
}

float add_float(float a, float b) {
  return a + b;
}

void check_3rdparty() {
  // ------------------ Tracy ---------------------
  ZoneScoped;

  // ------------------ Eigen ---------------------
  Eigen::MatrixXd matrix = Eigen::MatrixXd::Random(2, 2);
  std::cout << "Check Eigen:\n"
            << matrix << "\n";

  // ------------------ glog ---------------------
  // google::InitGoogleLogging("my_logger_name");
  LOG(INFO) << "Check glog";
  // google::ShutdownGoogleLogging();

  // ------------------ Boost ---------------------
  BOOST_PP_REPEAT(5, PRINT_NUMBER, ~);
}
}  // namespace wavemap::core