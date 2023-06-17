#include <google/protobuf/stubs/common.h>
#include <gtest/gtest.h>

// Gtest Environment that explicitly shuts down the protobuf library after all
// tests are completed. This is useful to avoid memory leak warnings related to
// objects that protobuf owns but does not explicitly free on shutdown.
class ProtobufEnvironment : public ::testing::Environment {
 public:
  void TearDown() override { google::protobuf::ShutdownProtobufLibrary(); }
};

int main(int argc, char** argv) {
  // Register the protobuf environment
  testing::AddGlobalTestEnvironment(new ProtobufEnvironment);
  // Initialize gtest and run all tests
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
