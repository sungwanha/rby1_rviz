#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

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
    TeleopIKNode(const std::string& urdf_path) : Node("teleop_ik_node"), urdf_path_(urdf_path) {
        // Pinocchio 모델 로드
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

        // 고정 조인트 설정 (팔만 움직임)
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

        set_init_pose();

        // ROS 2 설정
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/simdd/pose", 10, std::bind(&TeleopIKNode::teleop_callback, this, std::placeholders::_1));
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(16), std::bind(&TeleopIKNode::publish_joint_states, this));

        RCLCPP_INFO(this->get_logger(), "Teleop IK Node (C++) Started with URDF: %s", urdf_path_.c_str());
    }

private:
    void set_init_pose() {
        current_q_ = pinocchio::neutral(model_);
        std::map<std::string, double> init_values = {
            {"left_arm_3", -1.5708}, {"left_arm_6", -1.5708},
            {"right_arm_3", -1.5708}, {"right_arm_6", 1.5708}
        };
        for (const auto& [name, val] : init_values) {
            if (joint_limits_.count(name)) {
                current_q_[std::get<2>(joint_limits_[name])] = val;
            }
        }
    }

    void publish_joint_states() {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        for (int i = 1; i < model_.njoints; ++i) {
            if (model_.joints[i].nq() == 1) {
                msg.name.push_back(model_.names[i]);
                msg.position.push_back(current_q_[model_.joints[i].idx_q()]);
            }
        }
        publisher_->publish(msg);
    }

    void broadcast_target_tf(const std::string& parent, const std::string& child, const pinocchio::SE3& pose) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = parent;
        t.child_frame_id = child;
        t.transform.translation.x = pose.translation()[0];
        t.transform.translation.y = pose.translation()[1];
        t.transform.translation.z = pose.translation()[2];
        Eigen::Quaterniond q(pose.rotation());
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);
    }

    void teleop_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 32) return;
        const auto& data = msg->data;

        bool right_tracking = data[6] > 0.5;
        bool left_tracking = data[13] > 0.5;

        // 트래킹이 끊기면 다시 시작할 때 오프셋을 잡도록 플래그 초기화
        if (!right_tracking) first_tracking_right_ = true;
        if (!left_tracking) first_tracking_left_ = true;

        if (!right_tracking && !left_tracking) return;

        // OpenXR -> ROS 변환 로직
        auto get_quat = [&](double x, double y, double z, double w, double s, double c) {
            return Eigen::Quaterniond(w*c - z*s, x*c + y*s, y*c - x*s, z*c + w*s);
        };
        
        Eigen::Vector3d r_pos(-data[9], -data[7], data[8]);
        Eigen::Quaterniond q_r = get_quat(-data[26], -data[24], data[25], data[27], 0.70710678118, 0.70710678118);
        
        Eigen::Vector3d l_pos(-data[16], -data[14], data[15]);
        Eigen::Quaterniond q_l = get_quat(-data[30], -data[28], data[29], data[31], -0.70710678118, 0.70710678118);
        
        pinocchio::SE3 M_hmd_r(q_r.toRotationMatrix(), r_pos);
        pinocchio::SE3 M_hmd_l(q_l.toRotationMatrix(), l_pos);

        if (teleop_enabled_) {
            pinocchio::forwardKinematics(model_, data_, current_q_);
            pinocchio::updateFramePlacements(model_, data_);

            std::string head_frame = model_.existFrame("link_head_2") ? "link_head_2" : "head_1";
            auto head_id = model_.getFrameId(head_frame);
            pinocchio::SE3 oM_head = data_.oMf[head_id];

            // --- 상대 오프셋 계산 (Jump 방지) ---
            if (right_tracking && first_tracking_right_) {
                auto id_ee_r = model_.getFrameId("ee_right");
                M_offset_r_ = oM_head.actInv(data_.oMf[id_ee_r]) * M_hmd_r.inverse();
                first_tracking_right_ = false;
            }
            if (left_tracking && first_tracking_left_) {
                auto id_ee_l = model_.getFrameId("ee_left");
                M_offset_l_ = oM_head.actInv(data_.oMf[id_ee_l]) * M_hmd_l.inverse();
                first_tracking_left_ = false;
            }

            pinocchio::SE3 target_r = oM_head * M_offset_r_ * M_hmd_r;
            pinocchio::SE3 target_l = oM_head * M_offset_l_ * M_hmd_l;
            // ---------------------------------

            if (right_tracking) broadcast_target_tf("base", "target_right", target_r);
            if (left_tracking) broadcast_target_tf("base", "target_left", target_l);

            if (right_tracking && left_tracking) {
                solve_ik_dual("ee_right", target_r, "ee_left", target_l);
            } else if (right_tracking) {
                solve_ik_single("ee_right", target_r);
            } else if (left_tracking) {
                solve_ik_single("ee_left", target_l);
            }
        }
    }

    void solve_ik_single(const std::string& frame_name, const pinocchio::SE3& target_pose) {
        auto frame_id = model_.getFrameId(frame_name);
        Eigen::VectorXd q = current_q_;
        
        for (int i = 0; i < 10; ++i) {
            pinocchio::forwardKinematics(model_, data_, q);
            pinocchio::updateFramePlacements(model_, data_);
            
            pinocchio::SE3 dMf = target_pose.actInv(data_.oMf[frame_id]);
            Eigen::Matrix<double, 6, 1> err = pinocchio::log6(dMf).toVector();
            
            if (err.norm() < 1e-4) break;
            
            Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, model_.nv);
            J.setZero();
            pinocchio::computeFrameJacobian(model_, data_, q, frame_id, pinocchio::LOCAL, J);
            
            for (int idx : locked_v_indices_) J.col(idx).setZero();
            
            Eigen::MatrixXd JJt = J * J.transpose();
            JJt.diagonal().array() += damp_;
            Eigen::VectorXd v = -J.transpose() * JJt.ldlt().solve(err);
            
            q = pinocchio::integrate(model_, q, v * dt_);
            for (int j = 0; j < model_.nq; ++j) {
                q[j] = std::max(model_.lowerPositionLimit[j], std::min(model_.upperPositionLimit[j], q[j]));
            }
        }
        current_q_ = q;
    }

    void solve_ik_dual(const std::string& frame_r, const pinocchio::SE3& target_r, 
                       const std::string& frame_l, const pinocchio::SE3& target_l) {
        auto id_r = model_.getFrameId(frame_r);
        auto id_l = model_.getFrameId(frame_l);
        Eigen::VectorXd q = current_q_;

        for (int i = 0; i < 10; ++i) {
            pinocchio::forwardKinematics(model_, data_, q);
            pinocchio::updateFramePlacements(model_, data_);

            Eigen::Matrix<double, 12, 1> err;
            err.segment<6>(0) = pinocchio::log6(target_r.actInv(data_.oMf[id_r])).toVector();
            err.segment<6>(6) = pinocchio::log6(target_l.actInv(data_.oMf[id_l])).toVector();

            if (err.norm() < 1e-4) break;

            Eigen::Matrix<double, 12, Eigen::Dynamic> J(12, model_.nv);
            J.setZero();
            Eigen::Matrix<double, 6, Eigen::Dynamic> Jr(6, model_.nv), Jl(6, model_.nv);
            Jr.setZero(); Jl.setZero();
            pinocchio::computeFrameJacobian(model_, data_, q, id_r, pinocchio::LOCAL, Jr);
            pinocchio::computeFrameJacobian(model_, data_, q, id_l, pinocchio::LOCAL, Jl);
            
            for (int idx : locked_v_indices_) { Jr.col(idx).setZero(); Jl.col(idx).setZero(); }
            J.block(0, 0, 6, model_.nv) = Jr;
            J.block(6, 0, 6, model_.nv) = Jl;

            Eigen::MatrixXd JJt = J * J.transpose();
            JJt.diagonal().array() += damp_;
            Eigen::VectorXd v = -J.transpose() * JJt.ldlt().solve(err);

            q = pinocchio::integrate(model_, q, v * dt_);
            for (int j = 0; j < model_.nq; ++j) {
                q[j] = std::max(model_.lowerPositionLimit[j], std::min(model_.upperPositionLimit[j], q[j]));
            }
        }
        current_q_ = q;
    }

    std::string urdf_path_;
    pinocchio::Model model_;
    pinocchio::Data data_;
    Eigen::VectorXd current_q_;
    
    std::vector<std::string> joint_names_;
    std::map<std::string, std::tuple<double, double, int>> joint_limits_;
    std::vector<int> locked_v_indices_;

    bool teleop_enabled_ = true;
    double dt_ = 0.5;
    double damp_ = 1e-6;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::string urdf = "/home/hsw/rby1_rviz/model/model_v1.2.urdf";
    if (argc > 1) urdf = argv[1];
    auto node = std::make_shared<TeleopIKNode>(urdf);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
