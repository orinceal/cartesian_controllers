////////////////////////////////////////////////////////////////////////////////
// Copyright 2019 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
/*!\file    cartesian_compliance_controller.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_compliance_controller/cartesian_compliance_controller.h>

#include "cartesian_controller_base/Utility.h"
#include "controller_interface/controller_interface.hpp"

namespace cartesian_compliance_controller
{
CartesianComplianceController::CartesianComplianceController()
// Base constructor won't be called in diamond inheritance, so call that
// explicitly
: Base::CartesianControllerBase(),
  MotionBase::CartesianMotionController(),
  ForceBase::CartesianForceController()
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianComplianceController::on_init()
{
  using TYPE = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  if (MotionBase::on_init() != TYPE::SUCCESS || ForceBase::on_init() != TYPE::SUCCESS)
  {
    return TYPE::ERROR;
  }

  auto_declare<std::string>("compliance_ref_link", "");

  // declare virtual model spring parameters
  constexpr double default_lin_stiff = 500.0; // N/m
  constexpr double default_rot_stiff = 30.0;  // Nm/rad
  constexpr double default_lin_damp = 310.0;  // Ns/m
  constexpr double default_rot_damp = 75.0;  // Nms/rad
  constexpr double default_lin_inertial = 1.0; // kg
  constexpr double default_rot_inertial = 0.4; // kgm2
  // stiffness
  auto_declare<double>("compliance.trans_x.c", default_lin_stiff);
  auto_declare<double>("compliance.trans_y.c", default_lin_stiff);
  auto_declare<double>("compliance.trans_z.c", default_lin_stiff);
  auto_declare<double>("compliance.rot_x.c", default_rot_stiff);
  auto_declare<double>("compliance.rot_y.c", default_rot_stiff);
  auto_declare<double>("compliance.rot_z.c", default_rot_stiff);
  // damping
  auto_declare<double>("compliance.trans_x.k", default_lin_damp);
  auto_declare<double>("compliance.trans_y.k", default_lin_damp);
  auto_declare<double>("compliance.trans_z.k", default_lin_damp);
  auto_declare<double>("compliance.rot_x.k", default_rot_damp);
  auto_declare<double>("compliance.rot_y.k", default_rot_damp);
  auto_declare<double>("compliance.rot_z.k", default_rot_damp);
  // inertial
  auto_declare<double>("compliance.trans_x.I", default_lin_inertial);
  auto_declare<double>("compliance.trans_y.I", default_lin_inertial);
  auto_declare<double>("compliance.trans_z.I", default_lin_inertial);
  auto_declare<double>("compliance.rot_x.I", default_rot_inertial);
  auto_declare<double>("compliance.rot_y.I", default_rot_inertial);
  auto_declare<double>("compliance.rot_z.I", default_rot_inertial);


  return TYPE::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianComplianceController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  using TYPE = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  if (MotionBase::on_configure(previous_state) != TYPE::SUCCESS ||
      ForceBase::on_configure(previous_state) != TYPE::SUCCESS)
  {
    return TYPE::ERROR;
  }

  // Make sure compliance link is part of the robot chain
  m_compliance_ref_link = get_node()->get_parameter("compliance_ref_link").as_string();
  if (!Base::robotChainContains(m_compliance_ref_link))
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), m_compliance_ref_link
                                                    << " is not part of the kinematic chain from "
                                                    << Base::m_robot_base_link << " to "
                                                    << Base::m_end_effector_link);
    return TYPE::ERROR;
  }

  m_stiffness_diag[0] = get_node()->get_parameter("compliance.trans_x.c").as_double();
  m_stiffness_diag[1] = get_node()->get_parameter("compliance.trans_y.c").as_double();
  m_stiffness_diag[2] = get_node()->get_parameter("compliance.trans_z.c").as_double();
  m_stiffness_diag[3] = get_node()->get_parameter("compliance.rot_x.c").as_double();
  m_stiffness_diag[4] = get_node()->get_parameter("compliance.rot_y.c").as_double();
  m_stiffness_diag[5] = get_node()->get_parameter("compliance.rot_z.c").as_double();

  m_inertia_diag[0] = get_node()->get_parameter("compliance.trans_x.I").as_double();
  m_inertia_diag[1] = get_node()->get_parameter("compliance.trans_y.I").as_double();
  m_inertia_diag[2] = get_node()->get_parameter("compliance.trans_z.I").as_double();
  m_inertia_diag[3] = get_node()->get_parameter("compliance.rot_x.I").as_double();
  m_inertia_diag[4] = get_node()->get_parameter("compliance.rot_y.I").as_double();
  m_inertia_diag[5] = get_node()->get_parameter("compliance.rot_z.I").as_double();

  calculateCriticalDamping();
  // convert to base frame
  // Base::displayInBaseLink(m_stiffness, m_compliance_ref_link)

  m_param_handler = std:make_shared<rclcpp::ParameterEventHandler>(get_node());
  auto cb = [this](const rcl_interfaces::msg::ParameterEvent & event) {
    // logic to update m_stiffness and m_inertia when parameters change
  };
  m_cb_handle = m_param_handler->add_parameter_event_callback(cb);

  // Make sure sensor wrenches are interpreted correctly
  ForceBase::setFtSensorReferenceFrame(m_compliance_ref_link);

  return TYPE::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianComplianceController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  // Base::on_activation(..) will get called twice,
  // but that's fine.
  using TYPE = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  if (MotionBase::on_activate(previous_state) != TYPE::SUCCESS ||
      ForceBase::on_activate(previous_state) != TYPE::SUCCESS)
  {
    return TYPE::ERROR;
  }
  return TYPE::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianComplianceController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  using TYPE = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  if (MotionBase::on_deactivate(previous_state) != TYPE::SUCCESS ||
      ForceBase::on_deactivate(previous_state) != TYPE::SUCCESS)
  {
    return TYPE::ERROR;
  }
  return TYPE::SUCCESS;
}

controller_interface::return_type CartesianComplianceController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{

  KDL::Frame active_target;
  // get current target
  {
    std::lock_guard<std::mutex> lock(m_target_mutex);
    active_target = m_target_frame;
  }  
  // Synchronize the internal model and the real robot
  Base::m_ik_solver->synchronizeJointPositions(Base::m_joint_state_pos_handles);

  // Control the robot motion in such a way that the resulting net force
  // vanishes. This internal control needs some simulation time steps.
  for (int i = 0; i < Base::m_iterations; ++i)
  {
    // The internal 'simulation time' is deliberately independent of the outer
    // control cycle.
    auto internal_period = rclcpp::Duration::from_seconds(0.02);

    Base::m_ik_solver->updateKinematics();

    // Compute the net force
    ctrl::Vector6D error = computeComplianceError(active_target, internal_period);

    // Turn Cartesian error into joint motion
    Base::computeJointControlCmds(error, internal_period);
  }

  // Write final commands to the hardware interface
  Base::writeJointControlCmds();

  return controller_interface::return_type::OK;
}

ctrl::Vector6D CartesianComplianceController::computeComplianceError(const KDL::Frame& target_frame, const rclcpp::Duration& period)
{
  double dt = period.seconds();
  ctrl::Vector6D x_error = MotionBase::computeMotionError(target_frame);
  ctrl::Vector6D x_dot = Base::m_ik_solver->getEndEffectorVel();
  ctrl::Vector6D x_ddot = (x_dot - m_last_x_dot) / dt;
  
  ctrl::Vector6D net_force;
  for (int i=0; i < 6; ++i)
  {
    net_force[i] = (m_stiffness_diag[i] * x_error[i])
                 - (m_damping_diag[i] * x_dot[i])
                 - (m_inertia_diag[i] * x_ddot[i]);
  }

  ctrl::Vector6D net_force += ForceBase::computeForceError();

    // // Spring force in base orientation
    // Base::displayInBaseLink(m_stiffness, m_compliance_ref_link) * MotionBase::computeMotionError(target_frame)

    // // Sensor and target force in base orientation
    // + ForceBase::computeForceError();

  m_last_x_dot = x_dot;

  return net_force;
}

void CartesianComplianceController::calculateCriticalDamping(double zeta)
{
  for (int i = 0; i < 6; ++i)
  {
    // D = 2 * zeta * sqrt(M * K)
    m_damping_diag[i] = 2.0 * zeta * std::sqrt(stiffness[i] * inertia[i]);
  }
}

}  // namespace cartesian_compliance_controller

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cartesian_compliance_controller::CartesianComplianceController,
                       controller_interface::ControllerInterface)
