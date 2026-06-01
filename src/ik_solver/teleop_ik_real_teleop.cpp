#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <std_srvs/srv/set_bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <Eigen/Dense>
#include <vector>
#include <string>

using namespace std::chrono_literals;

class TeleopIKNode : public rclcpp::Node {
public:
    TeleopIKNode() : Node("teleop_ik_node") {
        RCLCPP_INFO(this->get_logger(), "노드 시작 중...");
        
        std::string pkg_share_path = ament_index_cpp::get_package_share_directory("rby1_teleop_cpp");
        std::string urdf_path = pkg_share_path + "/model/model_v1.2.urdf";
        
        pinocchio::urdf::buildModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);
        current_q_ = pinocchio::neutral(model_);

        // 고정할 관절 인덱스 (IK 시 속도를 0으로 고정)
        std::vector<std::string> locked_keywords = {"torso", "head", "wheel"};
        for (int i = 1; i < model_.njoints; ++i) {
            bool locked = false;
            for (const auto& kw : locked_keywords) {
                if (model_.names[i].find(kw) != std::string::npos) { locked = true; break; }
            }
            if (locked) {
                int nv = model_.joints[i].nv();
                int idx_v = model_.joints[i].idx_v();
                for (int v_i = 0; v_i < nv; ++v_i) locked_v_indices_.push_back(idx_v + v_i);
            }
        }

        setup_arm_info("ee_left", root_joint_l_, max_arm_length_l_);
        setup_arm_info("ee_right", root_joint_r_, max_arm_length_r_);

        // Pub/Sub/Srv
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        pub_left_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/leader/master_arm_joint_left/joint_trajectory", 10);
        pub_right_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/leader/master_arm_joint_right/joint_trajectory", 10);

        sub_pose_l_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/simdd/pose_left", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { left_pose_ = pose_to_se3(msg->pose); has_left_ = true; });
        sub_pose_r_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/simdd/pose_right", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { right_pose_ = pose_to_se3(msg->pose); has_right_ = true; });

        // 트리거 구독 (값이 들어오지 않아도 기본값 0.0 유지)
        sub_trig_l_ = this->create_subscription<sensor_msgs::msg::Joy>("/simdd/trigger_left", 10, [this](const sensor_msgs::msg::Joy::SharedPtr msg) { 
            if (!msg->axes.empty()) left_trig_val_ = 1.0 - msg->axes[0]; 
        });
        sub_trig_r_ = this->create_subscription<sensor_msgs::msg::Joy>("/simdd/trigger_right", 10, [this](const sensor_msgs::msg::Joy::SharedPtr msg) { 
            if (!msg->axes.empty()) right_trig_val_ = 1.0 - msg->axes[0]; 
        });

        sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            for (size_t i = 0; i < msg->name.size(); ++i) {
                if (model_.existJointName(msg->name[i])) {
                    auto j_id = model_.getJointId(msg->name[i]);
                    int idx = model_.joints[j_id].idx_q();
                    bool is_arm = (msg->name[i].find("arm") != std::string::npos);
                    if (!teleop_enabled_ || !is_arm) current_q_[idx] = msg->position[i];
                }
            }
        });

        srv_toggle_ = this->create_service<std_srvs::srv::SetBool>("toggle_teleop", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
            teleop_enabled_ = req->data;
            res->success = true;
            res->message = teleop_enabled_ ? "Teleop ON" : "Teleop OFF";
        });

        ik_timer_ = this->create_wall_timer(10ms, std::bind(&TeleopIKNode::update_ik, this));
        RCLCPP_INFO(this->get_logger(), "노드 초기화 완료.");
    }

private:
    void update_ik() {
        if (!teleop_enabled_) return;

        pinocchio::forwardKinematics(model_, data_, current_q_);
        pinocchio::updateFramePlacements(model_, data_);

        auto head_id = model_.getFrameId("zed2i_left_camera_optical_frame");
        pinocchio::SE3 oM_head = data_.oMf[head_id];

        if (has_right_) {
            pinocchio::SE3 target = (oM_head * right_pose_) * get_offset_se3(r_off_xyz_, r_off_rpy_);
            limit_arm_extension_relative(target, root_joint_r_, max_arm_length_r_);
            solve_ik_single("ee_right", target);
            broadcast_tf(target, "target_right");
        }

        if (has_left_) {
            pinocchio::SE3 target = (oM_head * left_pose_) * get_offset_se3(l_off_xyz_, l_off_rpy_);
            limit_arm_extension_relative(target, root_joint_l_, max_arm_length_l_);
            solve_ik_single("ee_left", target);
            broadcast_tf(target, "target_left");
        }

        publish_trajectories();
    }

    void solve_ik_single(const std::string& frame_name, const pinocchio::SE3& target) {
        auto frame_id = model_.getFrameId(frame_name);
        Eigen::VectorXd q = current_q_;
        const double ik_step = 0.4;
        const double damping = 0.05;

        for (int i = 0; i < 20; ++i) { 
            pinocchio::forwardKinematics(model_, data_, q);
            pinocchio::updateFramePlacements(model_, data_);
            pinocchio::SE3 dM = data_.oMf[frame_id].actInv(target);
            Eigen::Matrix<double, 6, 1> err = pinocchio::log6(dM).toVector();
            if (err.norm() < 1e-4) break;

            Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, model_.nv); J.setZero();
            pinocchio::computeFrameJacobian(model_, data_, q, frame_id, pinocchio::LOCAL, J);

            for (int idx : locked_v_indices_) J.col(idx).setZero();

            Eigen::MatrixXd J_inv = J.transpose() * (J * J.transpose() + Eigen::MatrixXd::Identity(6, 6) * damping).inverse();
            Eigen::VectorXd v = J_inv * err * ik_step;
            for (int idx : locked_v_indices_) v[idx] = 0.0;

            q = pinocchio::integrate(model_, q, v);
            for (int j = 1; j < model_.njoints; ++j) {
                if (model_.joints[j].nq() == 1) {
                    int idx = model_.joints[j].idx_q();
                    q[idx] = std::max(model_.lowerPositionLimit[idx], std::min(model_.upperPositionLimit[idx], q[idx]));
                }
            }
        }
        current_q_ = q;
    }

    void publish_trajectories() {
        auto msg_l = create_traj_msg(), msg_r = create_traj_msg();
        trajectory_msgs::msg::JointTrajectoryPoint pt_l, pt_r;
        pt_l.time_from_start = pt_r.time_from_start = rclcpp::Duration::from_nanoseconds(100000000);

        for (int i = 1; i < model_.njoints; ++i) {
            std::string name = model_.names[i];
            if (model_.joints[i].nq() == 1) {
                double pos = current_q_[model_.joints[i].idx_q()];
                if (name.find("left_arm") != std::string::npos) {
                    msg_l.joint_names.push_back(name); pt_l.positions.push_back(pos);
                } else if (name.find("right_arm") != std::string::npos) {
                    msg_r.joint_names.push_back(name); pt_r.positions.push_back(pos);
                }
            }
        }

        // --- 트리거 값 항상 추가 ---
        msg_l.joint_names.push_back("left_trigger");
        pt_l.positions.push_back(left_trig_val_);
        
        msg_r.joint_names.push_back("right_trigger");
        pt_r.positions.push_back(right_trig_val_);

        msg_l.points.push_back(pt_l); msg_r.points.push_back(pt_r);
        pub_left_->publish(msg_l); pub_right_->publish(msg_r);
    }

    void limit_arm_extension_relative(pinocchio::SE3& target, int root_id, double max_len) {
        if (root_id < 0) return;
        Eigen::Vector3d shoulder_pos = data_.oMi[root_id].translation();
        Eigen::Vector3d rel = target.translation() - shoulder_pos;
        if (rel.norm() > max_len) target.translation() = shoulder_pos + rel.normalized() * max_len;
    }

    void setup_arm_info(const std::string& ee_frame, int& root_id, double& max_len) {
        if (!model_.existFrame(ee_frame)) return;
        auto fid = model_.getFrameId(ee_frame);
        root_id = model_.frames[fid].parent; 
        max_len = 0.75;
    }

    pinocchio::SE3 pose_to_se3(const geometry_msgs::msg::Pose& p) {
        return pinocchio::SE3(Eigen::Quaterniond(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z).toRotationMatrix(), Eigen::Vector3d(p.position.x, p.position.y, p.position.z));
    }

    pinocchio::SE3 get_offset_se3(const Eigen::Vector3d& xyz, const Eigen::Vector3d& rpy) {
        double d2r = M_PI / 180.0;
        Eigen::Matrix3d R = (Eigen::AngleAxisd(rpy.z()*d2r, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(rpy.y()*d2r, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rpy.x()*d2r, Eigen::Vector3d::UnitX())).toRotationMatrix();
        return pinocchio::SE3(R, Eigen::Vector3d::Zero()) * pinocchio::SE3(Eigen::Matrix3d::Identity(), xyz);
    }

    void broadcast_tf(const pinocchio::SE3& pose, const std::string& child) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now(); t.header.frame_id = "base"; t.child_frame_id = child;
        t.transform.translation.x = pose.translation().x(); t.transform.translation.y = pose.translation().y(); t.transform.translation.z = pose.translation().z();
        Eigen::Quaterniond q(pose.rotation());
        t.transform.rotation.x = q.x(); t.transform.rotation.y = q.y(); t.transform.rotation.z = q.z(); t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);
    }

    trajectory_msgs::msg::JointTrajectory create_traj_msg() {
        trajectory_msgs::msg::JointTrajectory m; m.header.stamp = this->now(); return m;
    }

    pinocchio::Model model_; pinocchio::Data data_;
    Eigen::VectorXd current_q_;
    std::vector<int> locked_v_indices_;
    bool teleop_enabled_ = false, has_left_ = false, has_right_ = false;
    pinocchio::SE3 left_pose_, right_pose_;
    double left_trig_val_ = 0.0, right_trig_val_ = 0.0; // 기본값 0.0
    int root_joint_l_, root_joint_r_; double max_arm_length_l_, max_arm_length_r_;
    Eigen::Vector3d r_off_xyz_{-0.007, 0.003, -0.07}, r_off_rpy_{98.9, 0.3, 163.8};
    Eigen::Vector3d l_off_xyz_{-0.007, -0.005, -0.07}, l_off_rpy_{-97.5, 1.7, 15.3};

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_left_, pub_right_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_l_, sub_pose_r_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_trig_l_, sub_trig_r_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
    rclcpp::TimerBase::SharedPtr ik_timer_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_toggle_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopIKNode>());
    rclcpp::shutdown();
    return 0;
}