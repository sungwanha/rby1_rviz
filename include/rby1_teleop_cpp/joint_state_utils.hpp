#pragma once

#include <sensor_msgs/msg/joint_state.hpp>

#include <string>

namespace rby1_teleop_cpp::joint_state {

inline double positionOr(
    const sensor_msgs::msg::JointState& msg,
    const std::string& joint_name,
    double default_value) {
    for (size_t i = 0; i < msg.name.size() && i < msg.position.size(); ++i) {
        if (msg.name[i] == joint_name) {
            return msg.position[i];
        }
    }
    return default_value;
}

}  // namespace rby1_teleop_cpp::joint_state
