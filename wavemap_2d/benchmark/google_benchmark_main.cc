#include <benchmark/benchmark.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

int main(int argc, char** argv) {
  ::benchmark::Initialize(&argc, argv);
  google::ParseCommandLineFlags(&argc, &argv, false);
  FLAGS_logtostderr = true;
  FLAGS_minloglevel = 0;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  if (::benchmark::ReportUnrecognizedArguments(argc, argv)) {
    return 1;
  }
  ::benchmark::RunSpecifiedBenchmarks();
}
