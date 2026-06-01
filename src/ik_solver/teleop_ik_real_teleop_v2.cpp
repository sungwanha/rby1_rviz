#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp> 
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <std_srvs/srv/set_bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

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

        T_off_r_const_ = get_offset_se3(r_off_xyz_, r_off_rpy_);
        T_off_l_const_ = get_offset_se3(l_off_xyz_, l_off_rpy_);
        
        // URDF 로드
        std::string pkg_share_path = ament_index_cpp::get_package_share_directory("rby1_teleop_cpp");
        urdf_path_ = pkg_share_path + "/model/model_v1.2.urdf";
        
        pinocchio::urdf::buildModel(urdf_path_, model_);
        data_ = pinocchio::Data(model_);
        current_q_ = pinocchio::neutral(model_);

        // 관절 한계 및 인덱스 맵핑
        for (int i = 1; i < model_.njoints; ++i) {
            if (model_.joints[i].nq() == 1) {
                joint_limits_[model_.names[i]] = {model_.lowerPositionLimit[model_.joints[i].idx_q()], 
                                                 model_.upperPositionLimit[model_.joints[i].idx_q()], 
                                                 model_.joints[i].idx_q()};
            }
        }

        // 고정 조인트 인덱스 (Torso, Head 등)
        std::vector<std::string> locked_keywords = {"torso", "head", "wheel"};
        for (int i = 1; i < model_.njoints; ++i) {
            for (const auto& kw : locked_keywords) {
                if (model_.names[i].find(kw) != std::string::npos) {
                    for (int v_i = 0; v_i < model_.joints[i].nv(); ++v_i) 
                        locked_v_indices_.push_back(model_.joints[i].idx_v() + v_i);
                    break;
                }
            }
        }

        // 퍼블리셔
        pub_left_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/leader/master_arm_joint_left/joint_trajectory", 10);
        pub_right_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/leader/master_arm_joint_right/joint_trajectory", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 🔥 트리거 구독 (1.0 - value 로직 적용)
        sub_trig_l_ = this->create_subscription<sensor_msgs::msg::Joy>("/simdd/trigger_left", 10, [this](const sensor_msgs::msg::Joy::SharedPtr msg) { 
            // 입력(0~1) -> 반전(1~0)
            if (!msg->axes.empty()) left_trig_val_ = 1.0 - msg->axes[0]; 
        });
        sub_trig_r_ = this->create_subscription<sensor_msgs::msg::Joy>("/simdd/trigger_right", 10, [this](const sensor_msgs::msg::Joy::SharedPtr msg) { 
            // 입력(0~1) -> 반전(1~0)
            if (!msg->axes.empty()) right_trig_val_ = 1.0 - msg->axes[0]; 
        });

        // 상태 구독 및 서비스
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&TeleopIKNode::joint_state_callback, this, std::placeholders::_1));
        
        teleop_service_ = this->create_service<std_srvs::srv::SetBool>(
            "toggle_teleop", std::bind(&TeleopIKNode::teleop_toggle_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        sub_left_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/simdd/pose_left", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { left_pose_ = pose_to_se3(msg->pose); has_left_ = true; });
        sub_right_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/simdd/pose_right", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { right_pose_ = pose_to_se3(msg->pose); has_right_ = true; });

        ik_timer_ = this->create_wall_timer(20ms, std::bind(&TeleopIKNode::update_ik, this));
    }

private:
    pinocchio::SE3 T_off_r_const_;
    pinocchio::SE3 T_off_l_const_;
    
    pinocchio::SE3 get_offset_se3(const Eigen::Vector3d& xyz, const Eigen::Vector3d& rpy_deg) {
        double d2r = M_PI / 180.0;
        Eigen::Matrix3d R = (Eigen::AngleAxisd(rpy_deg.z() * d2r, Eigen::Vector3d::UnitZ()) *
                             Eigen::AngleAxisd(rpy_deg.y() * d2r, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(rpy_deg.x() * d2r, Eigen::Vector3d::UnitX())).toRotationMatrix();
        return pinocchio::SE3(R, Eigen::Vector3d::Zero()) * pinocchio::SE3(Eigen::Matrix3d::Identity(), xyz);
    }

    pinocchio::SE3 pose_to_se3(const geometry_msgs::msg::Pose& pose) {
        Eigen::Vector3d t(pose.position.x, pose.position.y, pose.position.z);
        Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        return pinocchio::SE3(q.normalized().toRotationMatrix(), t);
    }

    void teleop_toggle_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        teleop_enabled_ = req->data;
        res->success = true;
        res->message = teleop_enabled_ ? "Enabled" : "Disabled";
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (!teleop_enabled_) {
            for (size_t i = 0; i < msg->name.size(); ++i) {
                if (joint_limits_.count(msg->name[i])) {
                    int idx = std::get<2>(joint_limits_[msg->name[i]]);
                    current_q_[idx] = msg->position[i];
                }
            }
        }

        // 처음 데이터를 받았을 때 기준 자세(q_nominal_)로 현재 상태 저장
        if (!is_initialized_) {
            q_nominal_ = current_q_;
            is_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Nominal pose initialized from joint_states.");
        }
    }

    void update_ik() {
        if (!teleop_enabled_ || (!has_right_ && !has_left_)) return;

        pinocchio::forwardKinematics(model_, data_, current_q_);
        pinocchio::updateFramePlacements(model_, data_);

        auto head_id = model_.getFrameId("zed2i_left_camera_optical_frame");
        pinocchio::SE3 oM_head = data_.oMf[head_id];

        if (has_right_) {
            // 미리 계산된 오프셋 사용
            pinocchio::SE3 target = (oM_head * right_pose_) * T_off_r_const_;
            broadcast_target_tf("base", "target_right", target);
            solve_ik_single("ee_right", target);
        }
        
        if (has_left_) {
            pinocchio::SE3 target = (oM_head * left_pose_) * T_off_l_const_;
            broadcast_target_tf("base", "target_left", target);
            solve_ik_single("ee_left", target);
        }

        publish_joint_trajectories();
    }

    bool solve_ik_single(const std::string& frame, const pinocchio::SE3& target) {
        auto id = model_.getFrameId(frame); 
        Eigen::VectorXd q = current_q_;
        int nv = model_.nv; 

        std::vector<int> current_locked_v = locked_v_indices_;
        for (int j = 1; j < model_.njoints; ++j) {
            std::string j_name = model_.names[j];
            if ((frame == "ee_right" && j_name.find("left_arm") != std::string::npos) ||
                (frame == "ee_left" && j_name.find("right_arm") != std::string::npos)) {
                for (int v_i = 0; v_i < model_.joints[j].nv(); ++v_i) 
                    current_locked_v.push_back(model_.joints[j].idx_v() + v_i);
            }
        }

        for (int i = 0; i < 30; ++i) {
            pinocchio::forwardKinematics(model_, data_, q); 
            pinocchio::updateFramePlacements(model_, data_);
            const pinocchio::SE3 & dMi = data_.oMf[id];
            Eigen::Matrix<double, 6, 1> err = pinocchio::log6(dMi.actInv(target)).toVector();
            if (err.head<3>().norm() < 1e-4 && err.tail<3>().norm() < 1e-3) break; 

            Eigen::MatrixXd J(6, nv); J.setZero();
            pinocchio::computeFrameJacobian(model_, data_, q, id, pinocchio::LOCAL, J);
            for (int idx : current_locked_v) J.col(idx).setZero();
            
            double damp = 1e-3 + (err.head<3>().norm() * 0.5); 
            Eigen::MatrixXd JJt = J * J.transpose();
            JJt.diagonal().array() += std::min(damp, 0.1);
            Eigen::MatrixXd J_pinv = J.transpose() * JJt.inverse();

            // Null-space task로 q_nominal_ 유지
            Eigen::VectorXd v = J_pinv * err + (Eigen::MatrixXd::Identity(nv, nv) - J_pinv * J) * (0.05 * pinocchio::difference(model_, q, q_nominal_));
            for (int idx : current_locked_v) v[idx] = 0.0;
            
            double max_v = v.cwiseAbs().maxCoeff();
            if (max_v > 0.1) v *= (0.1 / max_v); 
            
            q = pinocchio::integrate(model_, q, v);
            for (int j = 1; j < model_.njoints; ++j) {
                if (model_.joints[j].nq() == 1) { 
                    int idx_q = model_.joints[j].idx_q();
                    q[idx_q] = std::max(model_.lowerPositionLimit[idx_q], std::min(model_.upperPositionLimit[idx_q], q[idx_q]));
                }
            }
        }
        if (!q.hasNaN()) { current_q_ = q; return true; }
        return false;
    }

    void publish_joint_trajectories() {
        auto msg_l = trajectory_msgs::msg::JointTrajectory();
        auto msg_r = trajectory_msgs::msg::JointTrajectory();
        msg_l.header.stamp = msg_r.header.stamp = this->now();

        trajectory_msgs::msg::JointTrajectoryPoint pt_l, pt_r;
        pt_l.time_from_start.nanosec = pt_r.time_from_start.nanosec = 100000000; // 0.1s
        
        for (int i = 1; i < model_.njoints; ++i) {
            if (model_.joints[i].nq() == 1) {
                double pos = current_q_[model_.joints[i].idx_q()];
                std::string name = model_.names[i];
                if (name.find("left_arm") != std::string::npos) {
                    msg_l.joint_names.push_back(name);
                    pt_l.positions.push_back(pos);
                } else if (name.find("right_arm") != std::string::npos) {
                    msg_r.joint_names.push_back(name);
                    pt_r.positions.push_back(pos);
                }
            }
        }

        // 🔥 반전된 트리거 값 추가 (1.0 ~ 0.0)
        msg_l.joint_names.push_back("left_trigger");
        pt_l.positions.push_back(left_trig_val_);
        
        msg_r.joint_names.push_back("right_trigger");
        pt_r.positions.push_back(right_trig_val_);

        msg_l.points.push_back(pt_l);
        msg_r.points.push_back(pt_r);

        pub_left_->publish(msg_l);
        pub_right_->publish(msg_r);
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
    pinocchio::Model model_; pinocchio::Data data_;
    Eigen::VectorXd current_q_, q_nominal_;
    std::map<std::string, std::tuple<double, double, int>> joint_limits_;
    std::vector<int> locked_v_indices_;
    bool teleop_enabled_ = false, has_left_ = false, has_right_ = false, is_initialized_ = false;
    pinocchio::SE3 left_pose_, right_pose_;

    double left_trig_val_ = 1.0, right_trig_val_ = 1.0; // 초기값 1.0 (안 당겼을 때)
    Eigen::Vector3d r_off_xyz_, r_off_rpy_, l_off_xyz_, l_off_rpy_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_left_, pub_right_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_trig_l_, sub_trig_r_; 
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_left_, sub_right_;
    rclcpp::TimerBase::SharedPtr ik_timer_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr teleop_service_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopIKNode>());
    rclcpp::shutdown();
    return 0;
}