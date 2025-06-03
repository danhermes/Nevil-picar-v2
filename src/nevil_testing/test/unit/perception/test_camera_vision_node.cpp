#include <gtest/gtest.h>

TEST(TestCameraVisionNode, TestInitialization) {
  // This is a minimal test that always passes
  ASSERT_TRUE(true);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
