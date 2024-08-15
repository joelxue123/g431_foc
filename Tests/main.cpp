#include <gtest/gtest.h>

TEST(SimpleTest, Addition) {
    EXPECT_EQ(2 + 2, 4);
}

TEST(SimpleTest, Multiplication) {
    EXPECT_EQ(3 * 3, 9);
}

int main(int argc, char **argv)
{

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
