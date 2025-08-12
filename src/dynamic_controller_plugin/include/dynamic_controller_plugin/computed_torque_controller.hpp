/******************************************************************************
                     ROS 2 computed_torque_controller Package
                            Computed Torque Controller
          Copyright (C) 2013..2025 Walter Fetter Lages <w.fetter@ieee.org>

        This program is free software: you can redistribute it and/or modify
        it under the terms of the GNU General Public License as published by
        the Free Software Foundation, either version 3 of the License, or
        (at your option) any later version.

        This program is distributed in the hope that it will be useful, but
        WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
        General Public License for more details.

        You should have received a copy of the GNU General Public License
        along with this program.  If not, see
        <http://www.gnu.org/licenses/>.
        
*******************************************************************************/

#ifndef COMPUTED_TORQUE_CONTROLLER__COMPUTED_TORQUE_CONTROLLER_HPP_
#define COMPUTED_TORQUE_CONTROLLER__COMPUTED_TORQUE_CONTROLLER_HPP_

#include <controller_interface/controller_interface.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

#include <realtime_tools/realtime_buffer.hpp>

#include <dynamic_controller_plugin/visibility_control.h>

namespace effort_controllers
{
    class ComputedTorqueController: public controller_interface::ControllerInterface
    {
    private:
        // Configuração das juntas
        std::vector<std::string> jointNames_;
        unsigned int nJoints_;

        // Subscrição para comandos
        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr sub_command_;

        // Modelo do robô e solver dinâmico
        std::string robotDescription_;          
        KDL::Chain chain_;      
        std::unique_ptr<KDL::ChainIdSolver_RNE> idsolver_;
                        
        // Estados das juntas
        KDL::JntArray q_;      // Posições atuais
        KDL::JntArray dq_;     // Velocidades atuais
        KDL::JntArray v_;      // Velocidades de referência calculadas
                
        // Trajetória de referência
        KDL::JntArray qr_;     // Posições desejadas
        KDL::JntArray dqr_;    // Velocidades desejadas
        KDL::JntArray ddqr_;   // Acelerações desejadas
                
        // Saída do controlador
        KDL::JntArray torque_; // Torques calculados
        
        // Forças externas
        KDL::Wrenches fext_;
                
        // Ganhos do controlador
        Eigen::MatrixXd Kp_;    // Matriz de ganho proporcional
        Eigen::MatrixXd Kd_;    // Matriz de ganho derivativo
        
        // Configuração de tempo real
        int priority_;
        
        // Buffer para pontos de referência (thread-safe)
        realtime_tools::RealtimeBuffer<std::shared_ptr<trajectory_msgs::msg::JointTrajectoryPoint>> referencePoint_;
        
        // Callbacks
        void commandCB(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr referencePoint);
        void robotDescriptionCB(const std_msgs::msg::String::SharedPtr robotDescription);
        
    public:
        COMPUTED_TORQUE_CONTROLLER_PUBLIC
        ComputedTorqueController(void);
        
        // Configuração das interfaces
        COMPUTED_TORQUE_CONTROLLER_PUBLIC
        controller_interface::InterfaceConfiguration command_interface_configuration(void) const override;

        COMPUTED_TORQUE_CONTROLLER_PUBLIC
        controller_interface::InterfaceConfiguration state_interface_configuration(void) const override;

        // Gerenciamento do ciclo de vida
        COMPUTED_TORQUE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_init(void) override;

        COMPUTED_TORQUE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

        COMPUTED_TORQUE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        COMPUTED_TORQUE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        // Atualização periódica do controlador
        COMPUTED_TORQUE_CONTROLLER_PUBLIC
        controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    };
}

#endif  // COMPUTED_TORQUE_CONTROLLER__COMPUTED_TORQUE_CONTROLLER_HPP_
