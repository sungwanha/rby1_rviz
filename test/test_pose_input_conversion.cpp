#include <gtest/gtest.h>

#include "rby1_teleop_cpp/pose_input_conversion.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace {

constexpr double kEps = 1e-9;

TEST(PoseInputConversion, ConvertsQuaternionPoseToSe3) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = 1.0;
    pose.position.y = 2.0;
    pose.position.z = 3.0;
    pose.orientation.w = 1.0;

    const auto se3 = rby1_teleop_cpp::pose_input::fromQuaternionPose(pose);

    EXPECT_NEAR(se3.translation().x(), 1.0, kEps);
    EXPECT_NEAR(se3.translation().y(), 2.0, kEps);
    EXPECT_NEAR(se3.translation().z(), 3.0, kEps);
    EXPECT_TRUE(se3.rotation().isApprox(Eigen::Matrix3d::Identity(), kEps));
}

TEST(PoseInputConversion, ConvertsSixDRowRepresentationToSe3) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0};

    pinocchio::SE3 se3;
    EXPECT_TRUE(rby1_teleop_cpp::pose_input::fromSixD(msg, se3));

    EXPECT_NEAR(se3.translation().x(), 1.0, kEps);
    EXPECT_NEAR(se3.translation().y(), 2.0, kEps);
    EXPECT_NEAR(se3.translation().z(), 3.0, kEps);
    EXPECT_TRUE(se3.rotation().isApprox(Eigen::Matrix3d::Identity(), kEps));
}

TEST(PoseInputConversion, RejectsInvalidSixDSize) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {1.0, 2.0, 3.0};

    pinocchio::SE3 se3;
    EXPECT_FALSE(rby1_teleop_cpp::pose_input::fromSixD(msg, se3));
}

TEST(PoseInputConversion, ConvertsRotationVectorToSe3) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {1.0, 2.0, 3.0, 0.0, 0.0, M_PI / 2.0};

    pinocchio::SE3 se3;
    EXPECT_TRUE(rby1_teleop_cpp::pose_input::fromRotationVector(msg, se3));

    Eigen::Matrix3d expected;
    expected = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ());
    EXPECT_NEAR(se3.translation().x(), 1.0, kEps);
    EXPECT_NEAR(se3.translation().y(), 2.0, kEps);
    EXPECT_NEAR(se3.translation().z(), 3.0, kEps);
    EXPECT_TRUE(se3.rotation().isApprox(expected, kEps));
}

TEST(PoseInputConversion, RejectsInvalidRotationVectorSize) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {1.0, 2.0, 3.0};

    pinocchio::SE3 se3;
    EXPECT_FALSE(rby1_teleop_cpp::pose_input::fromRotationVector(msg, se3));
}

TEST(PoseInputConversion, ConvertsSe3ToQuaternionPose) {
    pinocchio::SE3 se3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1.0, 2.0, 3.0));

    const auto pose = rby1_teleop_cpp::pose_input::toQuaternionPose(se3);

    EXPECT_NEAR(pose.position.x, 1.0, kEps);
    EXPECT_NEAR(pose.position.y, 2.0, kEps);
    EXPECT_NEAR(pose.position.z, 3.0, kEps);
    EXPECT_NEAR(pose.orientation.x, 0.0, kEps);
    EXPECT_NEAR(pose.orientation.y, 0.0, kEps);
    EXPECT_NEAR(pose.orientation.z, 0.0, kEps);
    EXPECT_NEAR(pose.orientation.w, 1.0, kEps);
}

TEST(PoseInputConversion, ConvertsSe3ToSixDRowRepresentation) {
    pinocchio::SE3 se3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1.0, 2.0, 3.0));

    const auto msg = rby1_teleop_cpp::pose_input::toSixD(se3);

    ASSERT_EQ(msg.data.size(), 9u);
    EXPECT_NEAR(msg.data[0], 1.0, kEps);
    EXPECT_NEAR(msg.data[1], 2.0, kEps);
    EXPECT_NEAR(msg.data[2], 3.0, kEps);
    EXPECT_NEAR(msg.data[3], 1.0, kEps);
    EXPECT_NEAR(msg.data[4], 0.0, kEps);
    EXPECT_NEAR(msg.data[5], 0.0, kEps);
    EXPECT_NEAR(msg.data[6], 0.0, kEps);
    EXPECT_NEAR(msg.data[7], 1.0, kEps);
    EXPECT_NEAR(msg.data[8], 0.0, kEps);
}

TEST(PoseInputConversion, ConvertsSe3ToRotationVectorPose) {
    Eigen::Matrix3d rotation;
    rotation = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ());
    pinocchio::SE3 se3(rotation, Eigen::Vector3d(1.0, 2.0, 3.0));

    const auto msg = rby1_teleop_cpp::pose_input::toRotationVector(se3);

    ASSERT_EQ(msg.data.size(), 6u);
    EXPECT_NEAR(msg.data[0], 1.0, kEps);
    EXPECT_NEAR(msg.data[1], 2.0, kEps);
    EXPECT_NEAR(msg.data[2], 3.0, kEps);
    EXPECT_NEAR(msg.data[3], 0.0, kEps);
    EXPECT_NEAR(msg.data[4], 0.0, kEps);
    EXPECT_NEAR(msg.data[5], M_PI / 2.0, kEps);
}

}  // namespace
