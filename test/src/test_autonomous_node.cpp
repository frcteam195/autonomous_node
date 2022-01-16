#include "test_autonomous_node.hpp"
#include "autonomous_node.hpp"
#include "ros/ros.h"

#include <gtest/gtest.h>

TEST(SampleTest, Test_Test)
{
    ASSERT_TRUE(1);
    ASSERT_FALSE(0);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_autonomous_node");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}