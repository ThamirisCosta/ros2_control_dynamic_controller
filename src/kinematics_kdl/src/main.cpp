#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <sstream>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/tree.hpp"
#include "kdl/chain.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::placeholders;

class IK : public rclcpp::Node
{
public:
    IK() : Node("inverse_kinematics"), first_iteration_(true), initial_x_(0.0), initial_y_(0.0), initial_z_(0.0)
    {
        // Inicializa o publisher para trajetória desejada
        desired_joint_trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/desired_joint_trajectory", 10);

        // Subscribers
        position_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/trajectory_points", 10, std::bind(&IK::positionCallback, this, _1));

        robot_description_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "robot_description",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
            std::bind(&IK::robotDescriptionCallback, this, _1));

        joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&IK::jointStateCallback, this, _1));

        RCLCPP_INFO(this->get_logger(), "IK node has been started.");
    }

private:
    void positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!solver_) {
            RCLCPP_WARN(this->get_logger(), "KDL solver not initialized yet.");
            return;
        }

        if (previous_joint_angles_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No previous joint angles available.");
            return;
        }

        const auto& position = msg->pose.position;
        RCLCPP_INFO(this->get_logger(), "Received position: x = %.2f, y = %.2f, z = %.2f", 
                   position.x, position.y, position.z);

        auto joint_angles = getJointAngles(position.x, position.y, position.z);
        if (joint_angles.empty()) {
            RCLCPP_WARN(this->get_logger(), "Failed to compute inverse kinematics. Using previous angles.");
            return;
        }

        publishDesiredTrajectory(joint_angles);
    }

    void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        const std::string urdf = msg->data;
        if (!kdl_parser::treeFromString(urdf, tree_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF from robot description.");
            return;
        }

        std::string last_link = getLastLink(tree_);
        if (last_link.empty() || !tree_.getChain("base_link", last_link, chain_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to extract KDL chain from base_link to %s", last_link.c_str());
            return;
        }

        // Initialize solvers
        solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_);
        fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);

        // Get joint names and initialize previous angles
        joint_names_ = getJointNames(chain_);
        previous_joint_angles_.assign(joint_names_.size(), 0.0);

        RCLCPP_INFO(this->get_logger(), "KDL chain created with %u joints.", chain_.getNrOfJoints());
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (joint_names_.empty()) {
            return;
        }

        for (size_t i = 0; i < msg->name.size(); ++i) {
            auto it = std::find(joint_names_.begin(), joint_names_.end(), msg->name[i]);
            if (it != joint_names_.end()) {
                size_t idx = std::distance(joint_names_.begin(), it);
                if (idx < previous_joint_angles_.size()) {
                    previous_joint_angles_[idx] = msg->position[i];
                }
            }
        }
        
        calculateCurrentPose();
    }

    std::vector<double> getJointAngles(double x, double y, double z)
    {
        if (!solver_ || previous_joint_angles_.empty()) {
            return {};
        }

        KDL::JntArray q_init(chain_.getNrOfJoints());
        for (size_t i = 0; i < previous_joint_angles_.size(); ++i) {
            q_init(i) = previous_joint_angles_[i];
        }

        KDL::Frame p_in(KDL::Vector(x, y, z));
        KDL::JntArray q_out(chain_.getNrOfJoints());
        
        int result = solver_->CartToJnt(q_init, p_in, q_out);
        if (result < 0) {
            RCLCPP_ERROR(this->get_logger(), "IK solver failed with error: %d", result);
            return {};
        }

        return std::vector<double>(q_out.data.data(), q_out.data.data() + q_out.data.size());
    }

    void calculateCurrentPose()
    {
        if (!fk_solver_ || previous_joint_angles_.empty()) {
            return;
        }

        KDL::JntArray joint_positions(chain_.getNrOfJoints());
        for (size_t i = 0; i < previous_joint_angles_.size(); ++i) {
            joint_positions(i) = previous_joint_angles_[i];
        }

        KDL::Frame current_pose;
        if (fk_solver_->JntToCart(joint_positions, current_pose) < 0) {
            RCLCPP_ERROR(this->get_logger(), "FK solver failed.");
            return;
        }

        const auto& pos = current_pose.p;
        if (first_iteration_) {
            first_iteration_ = false;
            initial_x_ = pos.x();
            initial_y_ = pos.y();
            initial_z_ = pos.z();
            RCLCPP_INFO(this->get_logger(), "Initial pose: x=%.3f, y=%.3f, z=%.3f", 
                        initial_x_, initial_y_, initial_z_);
        }
    }

    void publishDesiredTrajectory(const std::vector<double>& joint_angles)
    {
        auto trajectory_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
        trajectory_msg->header.stamp = this->now();
        trajectory_msg->joint_names = joint_names_;
        
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = joint_angles;
        
        // Você pode adicionar velocidades e acelerações desejadas aqui se disponíveis
        // point.velocities = {qdot1, qdot2, ..., qdotN};
        // point.accelerations = {qdotdot1, qdotdot2, ..., qdotdotN};
        
        point.time_from_start = rclcpp::Duration::from_seconds(0.1); // Tempo desejado para alcançar o ponto
        
        trajectory_msg->points.push_back(point);
        desired_joint_trajectory_publisher_->publish(*trajectory_msg);
    }

    static std::string getLastLink(const KDL::Tree& tree)
    {
        for (const auto& segment : tree.getSegments()) {
            if (segment.second.children.empty()) {
                return segment.first;
            }
        }
        return {};
    }

    static std::vector<std::string> getJointNames(const KDL::Chain& chain)
    {
        std::vector<std::string> names;
        for (size_t i = 0; i < chain.getNrOfSegments(); ++i) {
            const auto& joint = chain.getSegment(i).getJoint();
            if (joint.getType() != KDL::Joint::None) {
                names.push_back(joint.getName());
            }
        }
        return names;
    }

    // Publishers
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr desired_joint_trajectory_publisher_;
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;

    // KDL structures
    KDL::Tree tree_;
    KDL::Chain chain_;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> solver_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

    // State variables
    std::vector<std::string> joint_names_;
    std::vector<double> previous_joint_angles_;
    bool first_iteration_;
    double initial_x_, initial_y_, initial_z_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IK>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
