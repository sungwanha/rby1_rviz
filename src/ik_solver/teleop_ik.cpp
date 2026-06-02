#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "rby1_teleop_cpp/joint_state_utils.hpp"
#include "rby1_teleop_cpp/pose_input_conversion.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <map>
#include <chrono>

using namespace std::chrono_literals;

class TeleopIKNode : public rclcpp::Node {
public:
    TeleopIKNode() : Node("teleop_ik_node") {
        
        // --- [설정] 오프셋 파라미터 초기값 설정 ---
        r_off_xyz_ = {-0.0073, 0.0039, -0.0720}; 
        r_off_rpy_ = {98.99, 0.30, 163.89}; 

        l_off_xyz_ = {-0.0073, -0.0051, -0.0720}; 
        l_off_rpy_ = {-97.58, 1.71, 15.36};  

        last_print_time_ = this->now();
        pose_input_mode_ = this->declare_parameter<std::string>("pose_input_mode", "quaternion");

        const double D2R = M_PI / 180.0;

        // URDF 로드
        std::string pkg_share_path = ament_index_cpp::get_package_share_directory("rby1_teleop_cpp");
        urdf_path_ = pkg_share_path + "/model/model_v1.2.urdf";
        
        RCLCPP_INFO(this->get_logger(), "동적 로드된 URDF: %s", urdf_path_.c_str());

        pinocchio::urdf::buildModel(urdf_path_, model_);
        data_ = pinocchio::Data(model_);
        current_q_ = pinocchio::neutral(model_);

        // --- 안전한 초기 자세(Home Position) 강제 설정 ---
        for (int i = 1; i < model_.njoints; ++i) {
            std::string name = model_.names[i];
            int idx = model_.joints[i].idx_q();

            if (name == "left_arm_3")       current_q_[idx] = -130.0 * D2R; // -1.570
            else if (name == "left_arm_6")  current_q_[idx] = -90.0 * D2R; // -1.570
            else if (name == "right_arm_3") current_q_[idx] = -130.0 * D2R; // -1.570
            else if (name == "right_arm_6") current_q_[idx] =  90.0 * D2R; //  1.570
            // else if (name == "head_1")      current_q_[idx] =  40.0 * D2R; //  0.698132
            else if (name == "torso_1")      current_q_[idx] =  -22.8 * D2R;
            else if (name == "torso_3")      current_q_[idx] =  60.3 * D2R;
        }

        q_nominal_ = current_q_; 
        
        for (int i = 1; i < model_.njoints; ++i) {
            std::string name = model_.names[i];
            if (model_.joints[i].nq() == 1) {
                joint_names_.push_back(name);
                int idx_q = model_.joints[i].idx_q();
                double lower = model_.lowerPositionLimit[idx_q];
                double upper = model_.upperPositionLimit[idx_q];
                joint_limits_[name] = {lower, upper, idx_q};
            }
        }

        // 고정 조인트 설정
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

        // 팔 파라미터 계산
        auto calc_arm_params = [&](const std::string& ee_frame, int& root_joint, double& length) {
            length = 0.0;
            if (!model_.existFrame(ee_frame)) return;
            auto current_id = model_.getFrameId(ee_frame);
            auto joint_id = model_.frames[current_id].parent;
            root_joint = joint_id;
            while (joint_id > 0) {
                length += model_.jointPlacements[joint_id].translation().norm();
                int parent_id = model_.parents[joint_id];
                bool is_locked = false;
                if (parent_id > 0) {
                    for (int idx : locked_v_indices_) {
                        int v_start = model_.joints[parent_id].idx_v();
                        int nv = model_.joints[parent_id].nv();
                        if (idx >= v_start && idx < v_start + nv) { is_locked = true; break; }
                    }
                }
                if (parent_id == 0 || is_locked) break; 
                root_joint = parent_id; joint_id = parent_id;
            }
            length *= 0.95; 
        };
        calc_arm_params("ee_right", root_joint_r_, max_arm_length_r_);
        calc_arm_params("ee_left", root_joint_l_, max_arm_length_l_);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        pub_left_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/leader/master_arm_joint_left/joint_trajectory", 10);
        pub_right_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/leader/master_arm_joint_right/joint_trajectory", 10);

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&TeleopIKNode::joint_state_callback, this, std::placeholders::_1));
        sub_left_trigger_ = this->create_subscription<std_msgs::msg::Float64>(
            "/control/body/arm_left_controller/trigger", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                left_trigger_val_ = msg->data;
            });
        sub_right_trigger_ = this->create_subscription<std_msgs::msg::Float64>(
            "/control/body/arm_right_controller/trigger", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                right_trigger_val_ = msg->data;
            });
        teleop_service_ = this->create_service<std_srvs::srv::SetBool>(
            "toggle_teleop", std::bind(&TeleopIKNode::teleop_toggle_callback, this, std::placeholders::_1, std::placeholders::_2));
        nullspace_posture_service_ = this->create_service<std_srvs::srv::Trigger>(
            "update_nullspace_posture",
            std::bind(&TeleopIKNode::update_nullspace_posture_callback, this, std::placeholders::_1, std::placeholders::_2));
        configure_pose_input_subscriptions();

        ik_timer_ = this->create_wall_timer(20ms, std::bind(&TeleopIKNode::update_ik, this));
    }

private:
    Eigen::Vector3d r_off_xyz_, r_off_rpy_, l_off_xyz_, l_off_rpy_;

    pinocchio::SE3 get_offset_se3(const Eigen::Vector3d& xyz, const Eigen::Vector3d& rpy_deg) {
        double d2r = M_PI / 180.0;
        Eigen::Matrix3d R = (Eigen::AngleAxisd(rpy_deg.z() * d2r, Eigen::Vector3d::UnitZ()) *
                             Eigen::AngleAxisd(rpy_deg.y() * d2r, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(rpy_deg.x() * d2r, Eigen::Vector3d::UnitX())).toRotationMatrix();
        return pinocchio::SE3(R, Eigen::Vector3d::Zero()) * pinocchio::SE3(Eigen::Matrix3d::Identity(), xyz);
    }

    void teleop_toggle_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        teleop_enabled_ = req->data;
        res->success = true;
        res->message = teleop_enabled_ ? "Enabled" : "Disabled";
    }

    void update_nullspace_posture_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        if (!has_latest_joint_state_) {
            res->success = false;
            res->message = "Cannot update null-space posture: no /joint_states message received yet";
            return;
        }

        Eigen::VectorXd updated_nominal = q_nominal_;
        int updated_count = 0;
        for (size_t i = 0; i < latest_joint_state_.name.size() && i < latest_joint_state_.position.size(); ++i) {
            const auto it = joint_limits_.find(latest_joint_state_.name[i]);
            if (it == joint_limits_.end()) {
                continue;
            }
            updated_nominal[std::get<2>(it->second)] = latest_joint_state_.position[i];
            ++updated_count;
        }

        if (updated_count == 0) {
            res->success = false;
            res->message = "Cannot update null-space posture: /joint_states did not contain model joints";
            return;
        }

        q_nominal_ = updated_nominal;
        res->success = true;
        res->message = "Updated null-space posture from /joint_states with " + std::to_string(updated_count) + " joints";
        RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        latest_joint_state_ = *msg;
        has_latest_joint_state_ = true;

        left_trigger_val_ = rby1_teleop_cpp::joint_state::positionOr(*msg, "left_trigger", left_trigger_val_);
        right_trigger_val_ = rby1_teleop_cpp::joint_state::positionOr(*msg, "right_trigger", right_trigger_val_);

        if (teleop_enabled_) return; 
        for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
            if (joint_limits_.count(msg->name[i])) current_q_[std::get<2>(joint_limits_[msg->name[i]])] = msg->position[i];
        }
    }

    void configure_pose_input_subscriptions() {
        if (pose_input_mode_ == "quaternion") {
            sub_left_pose_ = this->create_subscription<geometry_msgs::msg::Pose>(
                "/control/body/arm_left_controller/pose", 10,
                [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
                    left_pose_ = rby1_teleop_cpp::pose_input::fromQuaternionPose(*msg);
                    has_left_ = true;
                });
            sub_right_pose_ = this->create_subscription<geometry_msgs::msg::Pose>(
                "/control/body/arm_right_controller/pose", 10,
                [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
                    right_pose_ = rby1_teleop_cpp::pose_input::fromQuaternionPose(*msg);
                    has_right_ = true;
                });
            RCLCPP_INFO(this->get_logger(), "Pose input mode: quaternion");
            return;
        }

        if (pose_input_mode_ == "six_d") {
            sub_left_array_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/control/body/arm_left_controller/pose_6d", 10,
                [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                    pinocchio::SE3 pose;
                    if (rby1_teleop_cpp::pose_input::fromSixD(*msg, pose)) {
                        left_pose_ = pose;
                        has_left_ = true;
                    } else {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                             "Ignoring left pose_6d: expected Float64MultiArray with 9 values");
                    }
                });
            sub_right_array_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/control/body/arm_right_controller/pose_6d", 10,
                [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                    pinocchio::SE3 pose;
                    if (rby1_teleop_cpp::pose_input::fromSixD(*msg, pose)) {
                        right_pose_ = pose;
                        has_right_ = true;
                    } else {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                             "Ignoring right pose_6d: expected Float64MultiArray with 9 values");
                    }
                });
            RCLCPP_INFO(this->get_logger(), "Pose input mode: six_d");
            return;
        }

        if (pose_input_mode_ == "rot_vec" || pose_input_mode_ == "axis_angle") {
            sub_left_array_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/control/body/arm_left_controller/pose_rot_vec", 10,
                [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                    pinocchio::SE3 pose;
                    if (rby1_teleop_cpp::pose_input::fromRotationVector(*msg, pose)) {
                        left_pose_ = pose;
                        has_left_ = true;
                    } else {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                             "Ignoring left pose_rot_vec: expected Float64MultiArray with 6 values");
                    }
                });
            sub_right_array_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/control/body/arm_right_controller/pose_rot_vec", 10,
                [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                    pinocchio::SE3 pose;
                    if (rby1_teleop_cpp::pose_input::fromRotationVector(*msg, pose)) {
                        right_pose_ = pose;
                        has_right_ = true;
                    } else {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                             "Ignoring right pose_rot_vec: expected Float64MultiArray with 6 values");
                    }
                });
            RCLCPP_INFO(this->get_logger(), "Pose input mode: %s", pose_input_mode_.c_str());
            return;
        }

        RCLCPP_WARN(this->get_logger(),
                    "Unknown pose_input_mode '%s'. Falling back to quaternion mode.",
                    pose_input_mode_.c_str());
        pose_input_mode_ = "quaternion";
        configure_pose_input_subscriptions();
    }

    void update_ik() {
        // 텔레옵 비활성화 시 리턴
        if (!teleop_enabled_ || (!has_right_ && !has_left_)) return;

        pinocchio::forwardKinematics(model_, data_, current_q_);
        pinocchio::updateFramePlacements(model_, data_);

        auto head_id = model_.getFrameId("zed2i_left_camera_optical_frame");
        pinocchio::SE3 oM_head = data_.oMf[head_id];
        pinocchio::SE3 T_off_r = get_offset_se3(r_off_xyz_, r_off_rpy_);
        pinocchio::SE3 T_off_l = get_offset_se3(l_off_xyz_, l_off_rpy_);

        // 🔥 3. 절대 위치 기반 타겟 생성 (오프셋 포함) 🔥
        if (has_right_) {
            pinocchio::SE3 target = (oM_head * right_pose_) * T_off_r;
            
            // 팔 길이 제한
            if (root_joint_r_ > 0) {
                Eigen::Vector3d rel = target.translation() - data_.oMi[root_joint_r_].translation();
                if (rel.norm() > max_arm_length_r_) {
                    target.translation() = data_.oMi[root_joint_r_].translation() + rel.normalized() * max_arm_length_r_;
                }
            }
            smoothed_target_r_ = target; 
            broadcast_target_tf("base", "target_right", smoothed_target_r_);
        }
        
        if (has_left_) {
            pinocchio::SE3 target = (oM_head * left_pose_) * T_off_l;
            
            // 팔 길이 제한
            if (root_joint_l_ > 0) {
                Eigen::Vector3d rel = target.translation() - data_.oMi[root_joint_l_].translation();
                if (rel.norm() > max_arm_length_l_) {
                    target.translation() = data_.oMi[root_joint_l_].translation() + rel.normalized() * max_arm_length_l_;
                }
            }
            smoothed_target_l_ = target;
            broadcast_target_tf("base", "target_left", smoothed_target_l_);
        }

        // 🔥 4. IK 풀기 (양팔 분리하여 상호 간섭 원천 차단) 🔥
        if (has_right_) solve_ik_single("ee_right", smoothed_target_r_);
        if (has_left_) solve_ik_single("ee_left", smoothed_target_l_);

        publish_joint_trajectories();
    }

    bool solve_ik_single(const std::string& frame, const pinocchio::SE3& target) {
        auto id = model_.getFrameId(frame); 
        Eigen::VectorXd q = current_q_;
        int nv = model_.nv; 
        double final_err_lin = 0.0;
        double final_err_ang = 0.0;

        // 🔥 [새로 추가] 상호 간섭 방지를 위한 동적 관절 잠금(Dynamic Lock) 🔥
        // 기본적으로 고정된 관절(torso, head 등) 목록을 복사해옵니다.
        std::vector<int> current_locked_v = locked_v_indices_;
        
        // 현재 계산하는 팔이 아닌 '반대쪽 팔'의 관절을 찾아서 잠금 목록에 추가합니다.
        for (int j = 1; j < model_.njoints; ++j) {
            std::string j_name = model_.names[j];
            bool lock_this_joint = false;
            
            // 오른팔을 계산할 때는 왼팔(left_arm)을 잠금
            if (frame == "ee_right" && j_name.find("left_arm") != std::string::npos) lock_this_joint = true;
            // 왼팔을 계산할 때는 오른팔(right_arm)을 잠금
            if (frame == "ee_left" && j_name.find("right_arm") != std::string::npos) lock_this_joint = true;
            
            if (lock_this_joint) {
                int idx_v = model_.joints[j].idx_v();
                int nv_j = model_.joints[j].nv();
                for (int v_i = 0; v_i < nv_j; ++v_i) {
                    current_locked_v.push_back(idx_v + v_i);
                }
            }
        }

        for (int i = 0; i < 30; ++i) {
            pinocchio::forwardKinematics(model_, data_, q); 
            pinocchio::updateFramePlacements(model_, data_);
            
            const pinocchio::SE3 & dMi = data_.oMf[id];
            Eigen::Matrix<double, 6, 1> err = pinocchio::log6(dMi.actInv(target)).toVector();
            
            final_err_lin = err.head<3>().norm();
            final_err_ang = err.tail<3>().norm();
            
            if (final_err_lin < 1e-4 && final_err_ang < 1e-3) break; 

            Eigen::MatrixXd J(6, nv); J.setZero();
            pinocchio::computeFrameJacobian(model_, data_, q, id, pinocchio::LOCAL, J);
            
            // 🔥 [수정] 기존 locked_v_indices_ 대신 current_locked_v 사용
            for (int idx : current_locked_v) J.col(idx).setZero();
            
            double adaptive_damp = 1e-3 + (final_err_lin * 0.5); 
            if (adaptive_damp > 0.1) adaptive_damp = 0.1; 
            
            Eigen::MatrixXd JJt = J * J.transpose();
            JJt.diagonal().array() += adaptive_damp;
            Eigen::MatrixXd J_pinv = J.transpose() * JJt.inverse();

            Eigen::VectorXd v_task = J_pinv * err;
            
            // 🔥 1. Null-space 기본 연산 (힘을 0.01 -> 0.05로 5배 증가)
            double gain_null = 0.05; 
            Eigen::VectorXd v_null_raw(nv);
            pinocchio::difference(model_, q, q_nominal_, v_null_raw);
            
            // 반대쪽 팔 간섭 방지용 Lock
            for (int idx : current_locked_v) v_null_raw[idx] = 0.0;
            
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(nv, nv);
            Eigen::VectorXd v_null_proj = (I - J_pinv * J) * (gain_null * v_null_raw); 

            // 🔥 2. 타겟 오차에 따른 부드러운 감쇠 (Soft-Decay)
            // 에러가 2cm 이내일 때는 100% 힘으로 팔꿈치를 예쁘게 모으고,
            // 에러가 커질수록(팔을 무리하게 뻗을수록) 널스페이스 힘을 부드럽게 줄임 (절대 갑자기 끄지 않음)
            double error_factor = 1.0;
            if (final_err_lin > 0.02) {
                error_factor = 0.02 / final_err_lin; 
            }
            v_null_proj *= error_factor;

            // 🔥 3. 튀는 현상 방지를 위한 널스페이스 한계치 설정
            // 아무리 널스페이스가 강해도 한 루프당 0.05 rad(약 3도) 이상 관절을 꺾지 못하게 제어
            if (v_null_proj.norm() > 0.05) {
                v_null_proj = v_null_proj.normalized() * 0.05;
            }

            // 최종 속도 합산
            Eigen::VectorXd v = v_task + v_null_proj;
            
            // 반대쪽 팔 속도 완전 차단
            for (int idx : current_locked_v) v[idx] = 0.0;
            
            // 🔥 4. 전체 속도 스케일링 (기존 유지)
            double max_v = v.cwiseAbs().maxCoeff();
            if (max_v > 0.1) {
                v = v * (0.1 / max_v); 
            }
            
            q = pinocchio::integrate(model_, q, v);
            
            for (int j = 1; j < model_.njoints; ++j) {
                if (model_.joints[j].nq() == 1) { 
                    int idx_q = model_.joints[j].idx_q();
                    q[idx_q] = std::max(model_.lowerPositionLimit[idx_q], std::min(model_.upperPositionLimit[idx_q], q[idx_q]));
                }
            }
        }

        if (!std::isnan(final_err_lin) && !std::isnan(final_err_ang) && !q.hasNaN()) {
            current_q_ = q;
            return true;
        }
        return false;
    }

    void publish_joint_trajectories() {
        auto msg_l = trajectory_msgs::msg::JointTrajectory(), msg_r = trajectory_msgs::msg::JointTrajectory();
        msg_l.header.stamp = msg_r.header.stamp = this->now();
        trajectory_msgs::msg::JointTrajectoryPoint pt_l, pt_r;
        pt_l.time_from_start.nanosec = pt_r.time_from_start.nanosec = 100000000;
        
        for (int i = 1; i < model_.njoints; ++i) {
            if (model_.joints[i].nq() == 1) {
                double pos = current_q_[model_.joints[i].idx_q()];
                if (model_.names[i].find("left_arm") != std::string::npos) { msg_l.joint_names.push_back(model_.names[i]); pt_l.positions.push_back(pos); }
                else if (model_.names[i].find("right_arm") != std::string::npos) { msg_r.joint_names.push_back(model_.names[i]); pt_r.positions.push_back(pos); }
            }
        }
        msg_l.joint_names.push_back("left_trigger"); pt_l.positions.push_back(left_trigger_val_);
        msg_r.joint_names.push_back("right_trigger"); pt_r.positions.push_back(right_trigger_val_);
        msg_l.points.push_back(pt_l); msg_r.points.push_back(pt_r);
        pub_left_->publish(msg_l); pub_right_->publish(msg_r);
    }

    void broadcast_target_tf(const std::string& p, const std::string& c, const pinocchio::SE3& pose) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now(); t.header.frame_id = p; t.child_frame_id = c;
        t.transform.translation.x = pose.translation()[0]; t.transform.translation.y = pose.translation()[1]; t.transform.translation.z = pose.translation()[2];
        Eigen::Quaterniond q(pose.rotation());
        t.transform.rotation.x = q.x(); t.transform.rotation.y = q.y(); t.transform.rotation.z = q.z(); t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);
    }

    std::string urdf_path_;
    std::string pose_input_mode_;
    pinocchio::Model model_; pinocchio::Data data_;
    Eigen::VectorXd current_q_, q_nominal_;
    std::vector<std::string> joint_names_;
    std::map<std::string, std::tuple<double, double, int>> joint_limits_;
    std::vector<int> locked_v_indices_;
    bool teleop_enabled_ = false, has_left_ = false, has_right_ = false, has_latest_joint_state_ = false;
    sensor_msgs::msg::JointState latest_joint_state_;
    pinocchio::SE3 left_pose_, right_pose_, smoothed_target_r_, smoothed_target_l_;
    rclcpp::Time last_print_time_;

    // 🔥 파라미터 튜닝 (발산 방지) 🔥
    double ik_step_ = 0.5;   // 기존 dt_ 대신 사용하는 IK 수렴 스텝 크기
    double damp_ = 1e-2;     // 특이점에서 발산을 막기 위한 Damping Factor
    
    double right_trigger_val_ = 0.0, left_trigger_val_ = 0.0;
    int root_joint_r_ = -1, root_joint_l_ = -1;
    double max_arm_length_r_ = 0.0, max_arm_length_l_ = 0.0;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_left_, pub_right_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_left_pose_, sub_right_pose_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_left_array_, sub_right_array_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_left_trigger_, sub_right_trigger_;
    rclcpp::TimerBase::SharedPtr ik_timer_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr teleop_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr nullspace_posture_service_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopIKNode>());
    rclcpp::shutdown();
    return 0;
}
