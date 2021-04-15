#include <gtest/gtest.h>
#include "test_ring.h"

TEST(test_ring, test_ring_negative_tests) {
    int rc = test_ring_negative_tests();
    EXPECT_EQ(rc, 0);
}

TEST(test_ring, test_ring_basic_ex) {
    int rc = test_ring_basic_ex();
    EXPECT_EQ(rc, 0);
}

TEST(test_ring, test_ring_with_exact_size) {
    int rc = test_ring_with_exact_size();
    EXPECT_EQ(rc, 0);
}
