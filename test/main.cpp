#include <gtest/gtest.h>

#include "lib.hpp"
#include "lib1.hpp"


int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
