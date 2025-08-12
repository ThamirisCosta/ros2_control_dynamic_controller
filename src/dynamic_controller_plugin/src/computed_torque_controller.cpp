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

#include <sys/mman.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <dynamic_controller_plugin/computed_torque_controller.hpp>

namespace effort_controllers
{       
        ComputedTorqueController::ComputedTorqueController(void):
                q_(0),dq_(0),v_(0),qr_(0),dqr_(0),ddqr_(0),torque_(0),fext_(0),referencePoint_(nullptr)
        {
        }

        controller_interface::CallbackReturn ComputedTorqueController::on_init(void)
        {
                try
                {
                        auto_declare<std::vector<std::string>>("joints",jointNames_);

                        auto_declare<std::string>("chain.root","origin_link");
                        auto_declare<std::string>("chain.tip", "tool_link");

                        auto_declare<double>("gravity.x",0.0);
                        auto_declare<double>("gravity.y",0.0);
                        auto_declare<double>("gravity.z",-9.8);

                        auto_declare<std::vector<double>>("Kp",std::vector<double>());
                        auto_declare<std::vector<double>>("Kd",std::vector<double>());

                        auto_declare<int>("priority",sched_get_priority_max(SCHED_FIFO));
                }
                catch(const std::exception &e)
                {
                        RCLCPP_ERROR_STREAM(get_node()->get_logger(),"Exception thrown in on_init() with message: " << e.what());
                        return controller_interface::CallbackReturn::ERROR;
                }

                rclcpp::QoS qos(rclcpp::KeepLast(1));
                qos.transient_local();
                auto robotDescriptionSubscriber=get_node()->create_subscription<std_msgs::msg::String>("robot_description",qos,std::bind(&ComputedTorqueController::robotDescriptionCB,this,std::placeholders::_1));
                while(robotDescription_.empty())
                {
                        RCLCPP_WARN_SKIPFIRST_THROTTLE(get_node()->get_logger(),*get_node()->get_clock(),1000,"Waiting for robot model on /robot_description.");
                        rclcpp::spin_some(get_node()->get_node_base_interface());
                }

                return controller_interface::CallbackReturn::SUCCESS;
        }

        controller_interface::CallbackReturn ComputedTorqueController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
        {
                jointNames_=get_node()->get_parameter("joints").as_string_array();
                if(jointNames_.empty())
                {
                        RCLCPP_ERROR(get_node()->get_logger(),"'joints' parameter was empty,");
                        return controller_interface::CallbackReturn::ERROR;
                }

                nJoints_=jointNames_.size();            

                sub_command_=get_node()->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>("command",1,
                        std::bind(&ComputedTorqueController::commandCB,this,std::placeholders::_1));

                while(robotDescription_.empty())
                        RCLCPP_WARN_SKIPFIRST_THROTTLE(get_node()->get_logger(),*get_node()->get_clock(),1000,"Waiting for robot model on /robot_description.");

                try
                {
                        KDL::Tree tree;         
                        if(!kdl_parser::treeFromString(robotDescription_,tree))
                                throw std::runtime_error("Failed to construct KDL tree.");

                        std::string chainRoot=get_node()->get_parameter("chain.root").as_string();
                        if(chainRoot.empty())
                                throw std::runtime_error("Could not find 'chain.root' parameter.");

                        std::string chainTip=get_node()->get_parameter("chain.tip").as_string();
                        if(chainTip.empty())
                                throw std::runtime_error("Could not find 'chain,tip' parameter.");

                        if(!tree.getChain(chainRoot,chainTip,chain_))
                                throw std::runtime_error("Failed to get chain from KDL tree.");

                        KDL::Vector g;
                        g[0]=get_node()->get_parameter("gravity.x").get_value<double>();
                        g[1]=get_node()->get_parameter("gravity.y").get_value<double>();
                        g[2]=get_node()->get_parameter("gravity.z").get_value<double>();

                        idsolver_=std::make_unique<KDL::ChainIdSolver_RNE>(chain_,g);
                
                        q_.resize(nJoints_);
                        dq_.resize(nJoints_);
                        v_.resize(nJoints_);
                        qr_.resize(nJoints_);
                        dqr_.resize(nJoints_);
                        ddqr_.resize(nJoints_);
                        torque_.resize(nJoints_);

                        fext_.resize(chain_.getNrOfSegments());

                        Kp_.resize(nJoints_,nJoints_);
                        Kd_.resize(nJoints_,nJoints_);

                        std::vector<double> KpVec=get_node()->get_parameter("Kp").as_double_array();
                        if(KpVec.empty())
                                throw std::runtime_error("No 'Kp' in controller.");
                        Kp_=Eigen::Map<Eigen::MatrixXd>(KpVec.data(),nJoints_,nJoints_).transpose();

                        std::vector<double> KdVec=get_node()->get_parameter("Kd").as_double_array();
                        if(KdVec.empty())
                                throw std::runtime_error("No 'Kd' in controller.");
                        Kd_=Eigen::Map<Eigen::MatrixXd>(KdVec.data(),nJoints_,nJoints_).transpose();
                }
                catch(const std::exception &e)
                {
                        RCLCPP_ERROR_STREAM(get_node()->get_logger(),"Exception thrown in on_confiture(): " << e.what());
                        return controller_interface::CallbackReturn::ERROR;
                }

                if(!get_node()->get_parameter("priority",priority_))
                        RCLCPP_WARN(get_node()->get_logger(),"No 'priority' configured for controller. Using highest possible priority.");

                return controller_interface::CallbackReturn::SUCCESS;
        }

        controller_interface::InterfaceConfiguration ComputedTorqueController::command_interface_configuration(void) const
        {
                controller_interface::InterfaceConfiguration config;
                config.type=controller_interface::interface_configuration_type::INDIVIDUAL;

                for(const auto &joint : jointNames_)
                        config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);

                return config;
        }

        controller_interface::InterfaceConfiguration ComputedTorqueController::state_interface_configuration(void) const
        {
                controller_interface::InterfaceConfiguration config;
                config.type=controller_interface::interface_configuration_type::INDIVIDUAL;

                for(const auto &joint : jointNames_)
                {
                        config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
                        config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
                }

                return config;
        }

        controller_interface::CallbackReturn ComputedTorqueController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
        {
                for(unsigned int i=0;i < nJoints_;i++)
                {
                        q_(i)=state_interfaces_[2*i].get_optional().value_or(q_(i));
                        dq_(i)=state_interfaces_[2*i+1].get_optional().value_or(dq_(i));
                }
                qr_=q_;
                dqr_=dq_;
                SetToZero(ddqr_);

                struct sched_param param;
                param.sched_priority=priority_;
                if(sched_setscheduler(0,SCHED_FIFO,&param) == -1)
                        RCLCPP_WARN(get_node()->get_logger(),"Failed to set real-time scheduler.");
                else if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1)
                        RCLCPP_WARN(get_node()->get_logger(),"Failed to lock memory.");

                return controller_interface::CallbackReturn::SUCCESS;
        }

        controller_interface::CallbackReturn ComputedTorqueController::on_deactivate(const rclcpp_lifecycle::State &/*previous_state*/)
        {
                for(unsigned int i=0;i < nJoints_;i++)
                        q_(i)=state_interfaces_[2*i].get_optional().value_or(q_(i));
                SetToZero(dqr_);
                SetToZero(ddqr_);

                for(auto &f : fext_) f.Zero();

                if(idsolver_->CartToJnt(q_,dqr_,ddqr_,fext_,torque_) < 0)
                        RCLCPP_ERROR(get_node()->get_logger(),"KDL inverse dynamics solver failed.");

                for(unsigned int i=0;i < nJoints_;i++)
                        if(!command_interfaces_[i].set_value(torque_(i)))
                                RCLCPP_ERROR(get_node()->get_logger(),"Can't set command value..");

                return controller_interface::CallbackReturn::SUCCESS;
        }

        controller_interface::return_type ComputedTorqueController::update(const rclcpp::Time &/*time*/,const rclcpp::Duration &/*period*/)
        {
                auto referencePoint=referencePoint_.readFromRT();
                if(referencePoint && *referencePoint)
                {
                        if((*referencePoint)->positions.size() == nJoints_)
                                qr_.data=Eigen::VectorXd::Map((*referencePoint)->positions.data(),nJoints_);
                                
                        if((*referencePoint)->velocities.size() == nJoints_)
                                dqr_.data=Eigen::VectorXd::Map((*referencePoint)->velocities.data(),nJoints_);
                                
                        if((*referencePoint)->accelerations.size() == nJoints_)
                                ddqr_.data=Eigen::VectorXd::Map((*referencePoint)->accelerations.data(),nJoints_);
                }

                for(unsigned int i=0;i < nJoints_;i++)
                {
                        q_(i)=state_interfaces_[2*i].get_optional().value_or(q_(i));
                        dq_(i)=state_interfaces_[2*i+1].get_optional().value_or(dq_(i));
                }
                for(auto &f : fext_) f.Zero();

                v_.data=ddqr_.data+Kp_*(qr_.data-q_.data)+Kd_*(dqr_.data-dq_.data);
                
                if(idsolver_->CartToJnt(q_,dq_,v_,fext_,torque_) < 0)
                        RCLCPP_ERROR(get_node()->get_logger(),"KDL inverse dynamics solver failed.");

                for(unsigned int i=0;i < nJoints_;i++)
                        if(!command_interfaces_[i].set_value(torque_(i)))
                                RCLCPP_ERROR(get_node()->get_logger(),"Can't set command value..");

                return controller_interface::return_type::OK;
        }

        void ComputedTorqueController::commandCB(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr referencePoint)
        {
                referencePoint_.writeFromNonRT(referencePoint);
        }

        void ComputedTorqueController::robotDescriptionCB(const std_msgs::msg::String::SharedPtr robotDescription)
        {
                robotDescription_=robotDescription->data;
        }
}

PLUGINLIB_EXPORT_CLASS(effort_controllers::ComputedTorqueController,controller_interface::ControllerInterface)
