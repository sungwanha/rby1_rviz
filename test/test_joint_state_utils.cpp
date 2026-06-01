#include <gtest/gtest.h>

#include "rby1_teleop_cpp/joint_state_utils.hpp"

#include <sensor_msgs/msg/joint_state.hpp>

TEST(JointStateUtils, FindsNamedPosition) {
    sensor_msgs::msg::JointState msg;
    msg.name = {"head_0", "left_trigger", "right_trigger"};
    msg.position = {0.1, 0.25, 0.75};

    EXPECT_DOUBLE_EQ(rby1_teleop_cpp::joint_state::positionOr(msg, "left_trigger", -1.0), 0.25);
    EXPECT_DOUBLE_EQ(rby1_teleop_cpp::joint_state::positionOr(msg, "right_trigger", -1.0), 0.75);
}

TEST(JointStateUtils, ReturnsDefaultWhenMissingOrPositionUnavailable) {
    sensor_msgs::msg::JointState missing;
    missing.name = {"left_trigger"};
    missing.position = {};

    EXPECT_DOUBLE_EQ(rby1_teleop_cpp::joint_state::positionOr(missing, "left_trigger", 1.0), 1.0);
    EXPECT_DOUBLE_EQ(rby1_teleop_cpp::joint_state::positionOr(missing, "right_trigger", 0.0), 0.0);
}
