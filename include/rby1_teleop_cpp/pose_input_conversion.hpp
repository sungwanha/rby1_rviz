#pragma once

#include <cmath>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace rby1_teleop_cpp::pose_input {

inline pinocchio::SE3 fromQuaternionPose(const geometry_msgs::msg::Pose& pose) {
    Eigen::Vector3d translation(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Quaterniond quaternion(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z);

    if (quaternion.norm() < 1e-12) {
        quaternion = Eigen::Quaterniond::Identity();
    }

    return pinocchio::SE3(quaternion.normalized().toRotationMatrix(), translation);
}

inline bool fromSixD(const std_msgs::msg::Float64MultiArray& msg, pinocchio::SE3& pose) {
    if (msg.data.size() != 9) {
        return false;
    }

    Eigen::Vector3d row0(msg.data[3], msg.data[4], msg.data[5]);
    Eigen::Vector3d row1(msg.data[6], msg.data[7], msg.data[8]);

    if (row0.norm() < 1e-12 || row1.norm() < 1e-12) {
        return false;
    }

    row0.normalize();
    row1 = row1 - row1.dot(row0) * row0;
    if (row1.norm() < 1e-12) {
        return false;
    }
    row1.normalize();
    const Eigen::Vector3d row2 = row0.cross(row1);

    Eigen::Matrix3d rotation;
    rotation.row(0) = row0.transpose();
    rotation.row(1) = row1.transpose();
    rotation.row(2) = row2.transpose();

    pose = pinocchio::SE3(
        rotation,
        Eigen::Vector3d(msg.data[0], msg.data[1], msg.data[2]));
    return true;
}

inline bool fromRotationVector(const std_msgs::msg::Float64MultiArray& msg, pinocchio::SE3& pose) {
    if (msg.data.size() != 6) {
        return false;
    }

    Eigen::Vector3d rot_vec(msg.data[3], msg.data[4], msg.data[5]);
    const double angle = rot_vec.norm();
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    if (angle >= 1e-12) {
        rotation = Eigen::AngleAxisd(angle, rot_vec / angle).toRotationMatrix();
    }

    pose = pinocchio::SE3(
        rotation,
        Eigen::Vector3d(msg.data[0], msg.data[1], msg.data[2]));
    return true;
}

inline geometry_msgs::msg::Pose toQuaternionPose(const pinocchio::SE3& pose) {
    geometry_msgs::msg::Pose msg;
    msg.position.x = pose.translation().x();
    msg.position.y = pose.translation().y();
    msg.position.z = pose.translation().z();

    Eigen::Quaterniond quaternion(pose.rotation());
    quaternion.normalize();
    msg.orientation.x = quaternion.x();
    msg.orientation.y = quaternion.y();
    msg.orientation.z = quaternion.z();
    msg.orientation.w = quaternion.w();
    return msg;
}

inline std_msgs::msg::Float64MultiArray toSixD(const pinocchio::SE3& pose) {
    std_msgs::msg::Float64MultiArray msg;
    const auto& rotation = pose.rotation();
    msg.data = {
        pose.translation().x(),
        pose.translation().y(),
        pose.translation().z(),
        rotation(0, 0),
        rotation(0, 1),
        rotation(0, 2),
        rotation(1, 0),
        rotation(1, 1),
        rotation(1, 2),
    };
    return msg;
}

inline std_msgs::msg::Float64MultiArray toRotationVector(const pinocchio::SE3& pose) {
    std_msgs::msg::Float64MultiArray msg;
    Eigen::AngleAxisd angle_axis(pose.rotation());
    Eigen::Vector3d rot_vec = Eigen::Vector3d::Zero();
    if (std::abs(angle_axis.angle()) >= 1e-12) {
        rot_vec = angle_axis.angle() * angle_axis.axis();
    }

    msg.data = {
        pose.translation().x(),
        pose.translation().y(),
        pose.translation().z(),
        rot_vec.x(),
        rot_vec.y(),
        rot_vec.z(),
    };
    return msg;
}

}  // namespace rby1_teleop_cpp::pose_input
