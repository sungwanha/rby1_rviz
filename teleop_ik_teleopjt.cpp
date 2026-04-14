#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <std_srvs/srv/set_bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/float32.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <map>

using namespace std::chrono_literals;

class TeleopIKNode : public rclcpp::Node {
public:
    TeleopIKNode() : Node("teleop_ik_node") {
        // 1. URDF 로드 및 Pinocchio 모델 설정
        std::string pkg_share_path = ament_index_cpp::get_package_share_directory("rby1_teleop_cpp");
        std::string urdf_path = pkg_share_path + "/model/model_v1.2.urdf";
        
        pinocchio::urdf::buildModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);
        current_q_ = pinocchio::neutral(model_); // 기본 0점 자세

        // 2. 팔 파라미터(End-effector 및 Root Joint) 자동 탐색
        setup_arm_info("ee_left", root_joint_l_, max_arm_length_l_);
        setup_arm_info("ee_right", root_joint_r_, max_arm_length_r_);

        // 3. 퍼블리셔 / 구독자 설정
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        pub_left_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/leader/master_arm_joint_left/joint_trajectory", 10);
        pub_right_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/leader/master_arm_joint_right/joint_trajectory", 10);

        // Pose 구독
        sub_pose_l_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/simdd/pose_left", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { left_pose_ = pose_to_se3(msg->pose); has_left_ = true; });
        sub_pose_r_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/simdd/pose_right", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { right_pose_ = pose_to_se3(msg->pose); has_right_ = true; });

        // 트리거 구독 (독립 토픽)
        sub_trig_l_ = this->create_subscription<std_msgs::msg::Float32>("/simdd/trigger_left", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) { left_trig_val_ = msg->data; });
        sub_trig_r_ = this->create_subscription<std_msgs::msg::Float32>("/simdd/trigger_right", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) { right_trig_val_ = msg->data; });

        // 상태 업데이트
        sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            if (!teleop_enabled_) {
                for (size_t i = 0; i < msg->name.size(); ++i) {
                    if (model_.existJointName(msg->name[i])) {
                        auto j_id = model_.getJointId(msg->name[i]);
                        current_q_[model_.joints[j_id].idx_q()] = msg->position[i];
                    }
                }
            }
        });

        srv_toggle_ = this->create_service<std_srvs::srv::SetBool>("toggle_teleop", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
            teleop_enabled_ = req->data;
            res->success = true;
            res->message = teleop_enabled_ ? "Teleop ON" : "Teleop OFF";
        });

        // 4. IK 타이머 (100Hz)
        ik_timer_ = this->create_wall_timer(10ms, std::bind(&TeleopIKNode::update_ik, this));
    }

private:
    void update_ik() {
        if (!teleop_enabled_) return;

        pinocchio::forwardKinematics(model_, data_, current_q_);
        pinocchio::updateFramePlacements(model_, data_);

        // 헤드 카메라 기준 좌표 변환 (zed2i_right_camera_optical_frame 등)
        auto head_id = model_.getFrameId("zed2i_right_camera_optical_frame");
        pinocchio::SE3 oM_head = data_.oMf[head_id];

        // --- IK 실행 (오른팔) ---
        if (has_right_) {
            pinocchio::SE3 target = (oM_head * right_pose_) * get_offset_se3(r_off_xyz_, r_off_rpy_);
            limit_arm_extension(target, root_joint_r_, max_arm_length_r_);
            solve_ik_single("ee_right", target);
            broadcast_tf(target, "target_right");
        }

        // --- IK 실행 (왼팔) ---
        if (has_left_) {
            pinocchio::SE3 target = (oM_head * left_pose_) * get_offset_se3(l_off_xyz_, l_off_rpy_);
            limit_arm_extension(target, root_joint_l_, max_arm_length_l_);
            solve_ik_single("ee_left", target);
            broadcast_tf(target, "target_left");
        }

        publish_trajectories();
    }

    void solve_ik_single(const std::string& frame_name, const pinocchio::SE3& target) {
        auto frame_id = model_.getFrameId(frame_name);
        Eigen::VectorXd q = current_q_;

        for (int i = 0; i < 10; ++i) { // 반복 횟수 최적화
            pinocchio::forwardKinematics(model_, data_, q);
            pinocchio::updateFramePlacements(model_, data_);
            
            pinocchio::SE3 dM = data_.oMf[frame_id].actInv(target);
            Eigen::Matrix<double, 6, 1> err = pinocchio::log6(dM).toVector();
            if (err.norm() < 1e-4) break;

            Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, model_.nv); J.setZero();
            pinocchio::computeFrameJacobian(model_, data_, q, frame_id, pinocchio::LOCAL, J);

            // Damped Least Squares
            Eigen::MatrixXd J_inv = J.transpose() * (J * J.transpose() + Eigen::MatrixXd::Identity(6, 6) * 0.01).inverse();
            q = pinocchio::integrate(model_, q, J_inv * err * 0.5);

            // **조인트 리미트 Clamping (안전 장치)**
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

        // 마지막 인덱스에 트리거 추가
        msg_l.joint_names.push_back("left_trigger"); pt_l.positions.push_back(left_trig_val_);
        msg_r.joint_names.push_back("right_trigger"); pt_r.positions.push_back(right_trig_val_);

        msg_l.points.push_back(pt_l); msg_r.points.push_back(pt_r);
        pub_left_->publish(msg_l); pub_right_->publish(msg_r);
    }

    // --- 유틸리티 함수들 ---
    void setup_arm_info(const std::string& ee_frame, int& root_id, double& max_len) {
        if (!model_.existFrame(ee_frame)) return;
        auto fid = model_.getFrameId(ee_frame);
        root_id = model_.frames[fid].parent;
        max_len = 0.8; // 기본 팔 길이 (계산 로직 간소화 가능)
    }

    void limit_arm_extension(pinocchio::SE3& target, int root_id, double max_len) {
        if (root_id < 0) return;
        Eigen::Vector3d rel = target.translation() - data_.oMi[root_id].translation();
        if (rel.norm() > max_len) target.translation() = data_.oMi[root_id].translation() + rel.normalized() * max_len;
    }

    pinocchio::SE3 pose_to_se3(const geometry_msgs::msg::Pose& p) {
        return pinocchio::SE3(Eigen::Quaterniond(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z).toRotationMatrix(), Eigen::Vector3d(p.position.x, p.position.y, p.position.z));
    }

    pinocchio::SE3 get_offset_se3(const Eigen::Vector3d& xyz, const Eigen::Vector3d& rpy) {
        double d2r = M_PI / 180.0;
        Eigen::Matrix3d R = (Eigen::AngleAxisd(rpy.z()*d2r, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(rpy.y()*d2r, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rpy.x()*d2r, Eigen::Vector3d::UnitX())).toRotationMatrix();
        return pinocchio::SE3(R, xyz);
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

    // 멤버 변수
    pinocchio::Model model_; pinocchio::Data data_;
    Eigen::VectorXd current_q_;
    bool teleop_enabled_ = false, has_left_ = false, has_right_ = false;
    pinocchio::SE3 left_pose_, right_pose_;
    double left_trig_val_ = 0.0, right_trig_val_ = 0.0;
    int root_joint_l_, root_joint_r_; double max_arm_length_l_, max_arm_length_r_;
    
    // 오프셋 (필요 시 수정)
    Eigen::Vector3d r_off_xyz_{-0.007, 0.003, -0.07}, r_off_rpy_{98.9, 0.3, 163.8};
    Eigen::Vector3d l_off_xyz_{-0.007, -0.005, -0.07}, l_off_rpy_{-97.5, 1.7, 15.3};

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_left_, pub_right_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_l_, sub_pose_r_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_trig_l_, sub_trig_r_;
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