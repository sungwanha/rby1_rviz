#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "rby1_teleop_cpp/joint_state_utils.hpp"
#include "rby1_teleop_cpp/pose_input_conversion.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <Eigen/Dense>
#include <cmath>
#include <map>
#include <string>
#include <tuple>

class ProprioPoseNode : public rclcpp::Node {
public:
    ProprioPoseNode() : Node("proprio_pose_node") {
        const std::string pkg_share_path = ament_index_cpp::get_package_share_directory("rby1_teleop_cpp");
        const std::string urdf_path = pkg_share_path + "/model/model_v1.2.urdf";

        pinocchio::urdf::buildModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);
        current_q_ = pinocchio::neutral(model_);

        for (int i = 1; i < model_.njoints; ++i) {
            if (model_.joints[i].nq() == 1) {
                joint_q_index_[model_.names[i]] = model_.joints[i].idx_q();
            }
        }

        head_frame_id_ = model_.getFrameId("zed2i_left_camera_optical_frame");
        left_ee_frame_id_ = model_.getFrameId("ee_left");
        right_ee_frame_id_ = model_.getFrameId("ee_right");
        left_offset_inv_ = get_offset_se3({-0.0073, -0.0051, -0.0720}, {-97.58, 1.71, 15.36}).inverse();
        right_offset_inv_ = get_offset_se3({-0.0073, 0.0039, -0.0720}, {98.99, 0.30, 163.89}).inverse();

        left_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/sensors/proprio/arm_left/pose", 10);
        right_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/sensors/proprio/arm_right/pose", 10);
        left_pose_6d_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/sensors/proprio/arm_left/pose_6d", 10);
        right_pose_6d_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/sensors/proprio/arm_right/pose_6d", 10);
        left_pose_rot_vec_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/sensors/proprio/arm_left/pose_rot_vec", 10);
        right_pose_rot_vec_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/sensors/proprio/arm_right/pose_rot_vec", 10);
        left_trigger_pub_ = this->create_publisher<std_msgs::msg::Float64>("/sensors/proprio/hand_left/trigger", 10);
        right_trigger_pub_ = this->create_publisher<std_msgs::msg::Float64>("/sensors/proprio/hand_right/trigger", 10);

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/sensors/proprio/body/joint_states", 10,
            std::bind(&ProprioPoseNode::joint_state_callback, this, std::placeholders::_1));
        left_hand_joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/sensors/proprio/hand_left/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                publish_left_trigger(triggerFromJointState(*msg, "left_trigger", left_trigger_value_));
            });
        right_hand_joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/sensors/proprio/hand_right/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                publish_right_trigger(triggerFromJointState(*msg, "right_trigger", right_trigger_value_));
            });

        RCLCPP_INFO(this->get_logger(), "Proprio pose node started with URDF: %s", urdf_path.c_str());
    }

private:
    pinocchio::SE3 get_offset_se3(const Eigen::Vector3d& xyz, const Eigen::Vector3d& rpy_deg) {
        const double d2r = M_PI / 180.0;
        Eigen::Matrix3d rotation =
            (Eigen::AngleAxisd(rpy_deg.z() * d2r, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(rpy_deg.y() * d2r, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(rpy_deg.x() * d2r, Eigen::Vector3d::UnitX()))
                .toRotationMatrix();
        return pinocchio::SE3(rotation, Eigen::Vector3d::Zero()) *
               pinocchio::SE3(Eigen::Matrix3d::Identity(), xyz);
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
            const auto it = joint_q_index_.find(msg->name[i]);
            if (it != joint_q_index_.end()) {
                current_q_[it->second] = msg->position[i];
            }
        }

        pinocchio::forwardKinematics(model_, data_, current_q_);
        pinocchio::updateFramePlacements(model_, data_);

        const pinocchio::SE3& oM_head = data_.oMf[head_frame_id_];
        const pinocchio::SE3 left_pose = oM_head.actInv(data_.oMf[left_ee_frame_id_]) * left_offset_inv_;
        const pinocchio::SE3 right_pose = oM_head.actInv(data_.oMf[right_ee_frame_id_]) * right_offset_inv_;

        publish_arm_poses(left_pose, right_pose);
    }

    void publish_arm_poses(const pinocchio::SE3& left_pose, const pinocchio::SE3& right_pose) {
        left_pose_pub_->publish(rby1_teleop_cpp::pose_input::toQuaternionPose(left_pose));
        right_pose_pub_->publish(rby1_teleop_cpp::pose_input::toQuaternionPose(right_pose));
        left_pose_6d_pub_->publish(rby1_teleop_cpp::pose_input::toSixD(left_pose));
        right_pose_6d_pub_->publish(rby1_teleop_cpp::pose_input::toSixD(right_pose));
        left_pose_rot_vec_pub_->publish(rby1_teleop_cpp::pose_input::toRotationVector(left_pose));
        right_pose_rot_vec_pub_->publish(rby1_teleop_cpp::pose_input::toRotationVector(right_pose));
    }

    double triggerFromJointState(
        const sensor_msgs::msg::JointState& msg,
        const std::string& default_joint_name,
        double previous_value) const {
        if (msg.position.empty()) {
            return previous_value;
        }
        if (msg.name.empty()) {
            return msg.position.front();
        }
        return rby1_teleop_cpp::joint_state::positionOr(msg, default_joint_name, msg.position.front());
    }

    void publish_left_trigger(double left_trigger) {
        left_trigger_value_ = left_trigger;
        std_msgs::msg::Float64 left_msg;
        left_msg.data = left_trigger_value_;
        left_trigger_pub_->publish(left_msg);
    }

    void publish_right_trigger(double right_trigger) {
        right_trigger_value_ = right_trigger;
        std_msgs::msg::Float64 right_msg;
        right_msg.data = right_trigger_value_;
        right_trigger_pub_->publish(right_msg);
    }

    pinocchio::Model model_;
    pinocchio::Data data_;
    Eigen::VectorXd current_q_;
    std::map<std::string, int> joint_q_index_;
    pinocchio::FrameIndex head_frame_id_;
    pinocchio::FrameIndex left_ee_frame_id_;
    pinocchio::FrameIndex right_ee_frame_id_;
    pinocchio::SE3 left_offset_inv_;
    pinocchio::SE3 right_offset_inv_;
    double left_trigger_value_ = 0.0;
    double right_trigger_value_ = 0.0;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_hand_joint_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr right_hand_joint_state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr left_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr right_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_pose_6d_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_pose_6d_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_pose_rot_vec_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_pose_rot_vec_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_trigger_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_trigger_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ProprioPoseNode>());
    rclcpp::shutdown();
    return 0;
}
