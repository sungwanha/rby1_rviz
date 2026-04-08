#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
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

class TeleopIKNode : public rclcpp::Node {
public:
    TeleopIKNode() : Node("teleop_ik_node") {
        
        std::string pkg_share_path = ament_index_cpp::get_package_share_directory("rby1_teleop_cpp");
        urdf_path_ = pkg_share_path + "/model/model_v1.2.urdf";
        
        RCLCPP_INFO(this->get_logger(), "동적 로드된 URDF: %s", urdf_path_.c_str());

        pinocchio::urdf::buildModel(urdf_path_, model_);
        data_ = pinocchio::Data(model_);
        current_q_ = pinocchio::neutral(model_);

        // 조인트 정보 및 제한 추출
        for (int i = 1; i < model_.njoints; ++i) {
            std::string name = model_.names[i];
            int nq = model_.joints[i].nq();
            if (nq == 1) {
                joint_names_.push_back(name);
                double lower = model_.lowerPositionLimit[model_.joints[i].idx_q()];
                double upper = model_.upperPositionLimit[model_.joints[i].idx_q()];
                joint_limits_[name] = {lower, upper, model_.joints[i].idx_q()};
            }
        }

        // 고정 조인트 설정 (몸통, 머리 등 고정)
        std::vector<std::string> locked_keywords = {"torso", "head", "wheel"};
        for (int i = 1; i < model_.njoints; ++i) {
            bool locked = false;
            for (const auto& kw : locked_keywords) {
                if (model_.names[i].find(kw) != std::string::npos) {
                    locked = true;
                    break;
                }
            }
            if (locked) {
                int nv = model_.joints[i].nv();
                int idx_v = model_.joints[i].idx_v();
                for (int v_i = 0; v_i < nv; ++v_i) {
                    locked_v_indices_.push_back(idx_v + v_i);
                }
            }
        }

        // ROS 2 통신 설정
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        pub_left_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/leader/master_arm_joint_left/joint_trajectory", 10);
        pub_right_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/leader/master_arm_joint_right/joint_trajectory", 10);

        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/simdd/pose", 10, std::bind(&TeleopIKNode::teleop_callback, this, std::placeholders::_1));
        
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&TeleopIKNode::joint_state_callback, this, std::placeholders::_1));

        teleop_service_ = this->create_service<std_srvs::srv::SetBool>(
            "toggle_teleop",
            std::bind(&TeleopIKNode::teleop_toggle_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Teleop IK Node Started. Waiting for '/toggle_teleop' service (True) to begin.");
    }

private:
    // --- 서비스 콜백: On/Off 제어 ---
    void teleop_toggle_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) 
    {
        teleop_enabled_ = request->data;
        if (teleop_enabled_) {
            first_tracking_right_ = true;
            first_tracking_left_ = true;
            RCLCPP_INFO(this->get_logger(), "Teleop ENABLED: Resetting offsets and starting IK.");
            response->message = "Enabled";
        } else {
            RCLCPP_INFO(this->get_logger(), "Teleop DISABLED: Syncing with robot state.");
            response->message = "Disabled";
        }
        response->success = true;
    }

    // --- Joint State 콜백: Teleop Off일 때 현재 로봇 자세 동기화 ---
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (teleop_enabled_) return; // Teleop On 상태에서는 로봇이 HMD를 따라가므로 동기화 중지

        for (size_t i = 0; i < msg->name.size(); ++i) {
            const std::string& name = msg->name[i];
            if (joint_limits_.count(name)) {
                int idx = std::get<2>(joint_limits_[name]);
                current_q_[idx] = msg->position[i];
            } else if (name == "left_trigger") {
                left_trigger_val_ = msg->position[i];
            } else if (name == "right_trigger") {
                right_trigger_val_ = msg->position[i];
            }
        }
    }

    // --- SE(3) 보간 함수: 보폭 제한 (Chasing 로직) ---
    pinocchio::SE3 limit_target_step(const pinocchio::SE3& current, const pinocchio::SE3& target) {
        pinocchio::SE3 dM = current.actInv(target);
        Eigen::Matrix<double, 6, 1> err = pinocchio::log6(dM).toVector();
        
        if (err.hasNaN()) return current;

        Eigen::Vector3d v = err.head<3>(); // 선형 이동
        Eigen::Vector3d w = err.tail<3>(); // 회전

        if (v.norm() > max_lin_step_) v = v.normalized() * max_lin_step_;
        if (w.norm() > max_ang_step_) w = w.normalized() * max_ang_step_;

        Eigen::Matrix<double, 6, 1> clamped_err;
        clamped_err << v, w;
        return current * pinocchio::exp6(clamped_err);
    }

    // --- 메인 텔레오퍼레이션 콜백 (데이터 수신 시 즉시 실행) ---
    void teleop_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 34) return;
        const auto& data = msg->data;

        // 트리거 값 갱신 (32: Right, 33: Left)
        right_trigger_val_ = data[32];
        left_trigger_val_ = data[33];

        bool right_tracking = data[6] > 0.5;
        bool left_tracking = data[13] > 0.5;

        // 트래킹이 끊기면 다음 연결 시 다시 오프셋을 초기화하도록 설정
        if (!right_tracking) first_tracking_right_ = true;
        if (!left_tracking) first_tracking_left_ = true;

        if (!teleop_enabled_ || (!right_tracking && !left_tracking)) return;

        // OpenXR -> ROS 변환 로직 (쿼터니언 정규화 포함)
        auto get_quat = [&](double x, double y, double z, double w, double s, double c) {
            Eigen::Quaterniond q(w*c - z*s, x*c + y*s, y*c - x*s, z*c + w*s);
            if (q.norm() < 1e-6) return Eigen::Quaterniond::Identity();
            q.normalize();
            return q;
        };

        Eigen::Vector3d r_pos(-data[9], -data[7], data[8]);
        Eigen::Quaterniond q_r = get_quat(-data[26], -data[24], data[25], data[27], 0.7071, 0.7071);
        Eigen::Vector3d l_pos(-data[16], -data[14], data[15]);
        Eigen::Quaterniond q_l = get_quat(-data[30], -data[28], data[29], data[31], -0.7071, 0.7071);

        pinocchio::SE3 M_hmd_r(q_r.toRotationMatrix(), r_pos);
        pinocchio::SE3 M_hmd_l(q_l.toRotationMatrix(), l_pos);

        // 현재 로봇 상태 업데이트
        pinocchio::forwardKinematics(model_, data_, current_q_);
        pinocchio::updateFramePlacements(model_, data_);

        std::string head_ref_frame = "zed2i_right_camera_optical_frame";
        auto head_id = model_.getFrameId(head_ref_frame);
        pinocchio::SE3 oM_head = data_.oMf[head_id];

        // --- 오프셋 및 보간 타겟 초기화 ---
        if (right_tracking && first_tracking_right_) {
            auto id_ee_r = model_.getFrameId("ee_right");
            
            initial_hmd_r_ = M_hmd_r;
            initial_ee_r_ = oM_head.actInv(data_.oMf[id_ee_r]); // Head 프레임 기준 초기 로봇 EE
            
            smoothed_target_r_ = data_.oMf[id_ee_r];
            first_tracking_right_ = false;
        }
        if (left_tracking && first_tracking_left_) {
            auto id_ee_l = model_.getFrameId("ee_left");
            
            initial_hmd_l_ = M_hmd_l;
            initial_ee_l_ = oM_head.actInv(data_.oMf[id_ee_l]); // Head 프레임 기준 초기 로봇 EE
            
            smoothed_target_l_ = data_.oMf[id_ee_l];
            first_tracking_left_ = false;
        }

        // --- 변위(Delta) 기반 타겟 계산 ---
        pinocchio::SE3 target_r_raw = smoothed_target_r_;
        if (right_tracking && !first_tracking_right_) {
            // 위치 변위 및 회전 변위 분리 계산
            Eigen::Vector3d delta_p = M_hmd_r.translation() - initial_hmd_r_.translation();
            Eigen::Matrix3d delta_R = M_hmd_r.rotation() * initial_hmd_r_.rotation().transpose();

            pinocchio::SE3 target_r_relative;
            target_r_relative.translation() = initial_ee_r_.translation() + delta_p;
            target_r_relative.rotation() = delta_R * initial_ee_r_.rotation();

            // Head 프레임을 곱해 Base 프레임 기준 타겟으로 변환
            target_r_raw = oM_head * target_r_relative;
        }

        pinocchio::SE3 target_l_raw = smoothed_target_l_;
        if (left_tracking && !first_tracking_left_) {
            // 위치 변위 및 회전 변위 분리 계산
            Eigen::Vector3d delta_p = M_hmd_l.translation() - initial_hmd_l_.translation();
            Eigen::Matrix3d delta_R = M_hmd_l.rotation() * initial_hmd_l_.rotation().transpose();

            pinocchio::SE3 target_l_relative;
            target_l_relative.translation() = initial_ee_l_.translation() + delta_p;
            target_l_relative.rotation() = delta_R * initial_ee_l_.rotation();

            // Head 프레임을 곱해 Base 프레임 기준 타겟으로 변환
            target_l_raw = oM_head * target_l_relative;
        }

        // 보간 적용 (Rate Limiting)
        if (right_tracking) smoothed_target_r_ = limit_target_step(smoothed_target_r_, target_r_raw);
        if (left_tracking) smoothed_target_l_ = limit_target_step(smoothed_target_l_, target_l_raw);

        // 시각화 (TF Broadcaster)
        if (right_tracking) broadcast_target_tf("base", "target_right", smoothed_target_r_);
        if (left_tracking) broadcast_target_tf("base", "target_left", smoothed_target_l_);

        // IK 풀기
        if (right_tracking && left_tracking) {
            solve_ik_dual("ee_right", smoothed_target_r_, "ee_left", smoothed_target_l_);
        } else if (right_tracking) {
            solve_ik_single("ee_right", smoothed_target_r_);
        } else if (left_tracking) {
            solve_ik_single("ee_left", smoothed_target_l_);
        }

        // --- 이벤트 기반 발행: 계산 끝나자마자 즉시 쏘기 ---
        publish_joint_trajectories();
    }

    void publish_joint_trajectories() {
        auto msg_l = trajectory_msgs::msg::JointTrajectory();
        auto msg_r = trajectory_msgs::msg::JointTrajectory();
        msg_l.header.stamp = msg_r.header.stamp = this->now();

        trajectory_msgs::msg::JointTrajectoryPoint pt_l, pt_r;
        pt_l.time_from_start.nanosec = pt_r.time_from_start.nanosec = 100000000;

        for (int i = 1; i < model_.njoints; ++i) {
            if (model_.joints[i].nq() == 1) {
                std::string name = model_.names[i];
                double pos = current_q_[model_.joints[i].idx_q()];

                if (name.find("left_arm") != std::string::npos) {
                    msg_l.joint_names.push_back(name);
                    pt_l.positions.push_back(pos);
                } else if (name.find("right_arm") != std::string::npos) {
                    msg_r.joint_names.push_back(name);
                    pt_r.positions.push_back(pos);
                }
            }
        }
        // 트리거 추가
        msg_l.joint_names.push_back("left_trigger");
        pt_l.positions.push_back(left_trigger_val_);
        msg_r.joint_names.push_back("right_trigger");
        pt_r.positions.push_back(right_trigger_val_);

        msg_l.points.push_back(pt_l); msg_r.points.push_back(pt_r);
        pub_left_->publish(msg_l); pub_right_->publish(msg_r);
    }

    void solve_ik_single(const std::string& frame_name, const pinocchio::SE3& target_pose) {
        auto frame_id = model_.getFrameId(frame_name);
        Eigen::VectorXd q = current_q_;
        for (int i = 0; i < 15; ++i) {
            pinocchio::forwardKinematics(model_, data_, q);
            pinocchio::updateFramePlacements(model_, data_);
            pinocchio::SE3 dMf = data_.oMf[frame_id].actInv(target_pose);
            Eigen::Matrix<double, 6, 1> err = pinocchio::log6(dMf).toVector();
            if (err.norm() < 1e-4 || err.hasNaN()) break;
            Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, model_.nv); J.setZero();
            pinocchio::computeFrameJacobian(model_, data_, q, frame_id, pinocchio::ReferenceFrame::LOCAL, J);
            for (int idx : locked_v_indices_) J.col(idx).setZero();
            Eigen::MatrixXd JJt = J * J.transpose();
            JJt.diagonal().array() += damp_;
            Eigen::VectorXd v = J.transpose() * JJt.ldlt().solve(err);
            q = pinocchio::integrate(model_, q, v * dt_);
            for (int j = 0; j < model_.nq; ++j) q[j] = std::max(model_.lowerPositionLimit[j], std::min(model_.upperPositionLimit[j], q[j]));
        }
        current_q_ = q;
    }

    void solve_ik_dual(const std::string& frame_r, const pinocchio::SE3& target_r, 
                       const std::string& frame_l, const pinocchio::SE3& target_l) {
        auto id_r = model_.getFrameId(frame_r); auto id_l = model_.getFrameId(frame_l);
        Eigen::VectorXd q = current_q_;
        for (int i = 0; i < 15; ++i) {
            pinocchio::forwardKinematics(model_, data_, q);
            pinocchio::updateFramePlacements(model_, data_);
            Eigen::Matrix<double, 12, 1> err;
            err.segment<6>(0) = pinocchio::log6(data_.oMf[id_r].actInv(target_r)).toVector();
            err.segment<6>(6) = pinocchio::log6(data_.oMf[id_l].actInv(target_l)).toVector();
            if (err.norm() < 1e-4 || err.hasNaN()) break;
            Eigen::Matrix<double, 12, Eigen::Dynamic> J(12, model_.nv); J.setZero();
            Eigen::Matrix<double, 6, Eigen::Dynamic> Jr(6, model_.nv), Jl(6, model_.nv);
            Jr.setZero(); Jl.setZero();
            pinocchio::computeFrameJacobian(model_, data_, q, id_r, pinocchio::ReferenceFrame::LOCAL, Jr);
            pinocchio::computeFrameJacobian(model_, data_, q, id_l, pinocchio::ReferenceFrame::LOCAL, Jl);
            for (int idx : locked_v_indices_) { Jr.col(idx).setZero(); Jl.col(idx).setZero(); }
            J.block(0, 0, 6, model_.nv) = Jr; J.block(6, 0, 6, model_.nv) = Jl;
            Eigen::MatrixXd JJt = J * J.transpose();
            JJt.diagonal().array() += damp_;
            Eigen::VectorXd v = J.transpose() * JJt.ldlt().solve(err);
            q = pinocchio::integrate(model_, q, v * dt_);
            for (int j = 0; j < model_.nq; ++j) q[j] = std::max(model_.lowerPositionLimit[j], std::min(model_.upperPositionLimit[j], q[j]));
        }
        current_q_ = q;
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
    pinocchio::Model model_;
    pinocchio::Data data_;
    Eigen::VectorXd current_q_;
    std::vector<std::string> joint_names_;
    std::map<std::string, std::tuple<double, double, int>> joint_limits_;
    std::vector<int> locked_v_indices_;

    bool teleop_enabled_ = false;
    bool first_tracking_right_ = true, first_tracking_left_ = true;
    
    // 수정된 부분: 조종기 및 로봇 EE의 초기 포즈 저장 변수
    pinocchio::SE3 initial_hmd_r_, initial_ee_r_;
    pinocchio::SE3 initial_hmd_l_, initial_ee_l_;
    pinocchio::SE3 smoothed_target_r_, smoothed_target_l_;
    
    double dt_ = 0.5, damp_ = 1e-6;
    double max_lin_step_ = 0.03, max_ang_step_ = 0.1; // 보간 제한값
    double right_trigger_val_ = 0.0, left_trigger_val_ = 0.0;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_left_, pub_right_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr teleop_service_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopIKNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}