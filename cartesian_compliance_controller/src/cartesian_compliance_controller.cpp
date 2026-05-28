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
  // constexpr double default_lin_stiff = 500.0; // N/m
  // constexpr double default_rot_stiff = 30.0;  // Nm/rad
  // constexpr double default_lin_damp = 310.0;  // Ns/m
  // constexpr double default_rot_damp = 75.0;  // Nms/rad
  // constexpr double default_lin_inertial = 1.0; // kg
  // constexpr double default_rot_inertial = 0.4; // kgm2
  // stiffness
  // auto_declare<double>("compliance.trans_x.c", default_lin_stiff);
  // auto_declare<double>("compliance.trans_y.c", default_lin_stiff);
  // auto_declare<double>("compliance.trans_z.c", default_lin_stiff);
  // auto_declare<double>("compliance.rot_x.c", default_rot_stiff);
  // auto_declare<double>("compliance.rot_y.c", default_rot_stiff);
  // auto_declare<double>("compliance.rot_z.c", default_rot_stiff);
  // // damping
  // auto_declare<double>("compliance.trans_x.k", default_lin_damp);
  // auto_declare<double>("compliance.trans_y.k", default_lin_damp);
  // auto_declare<double>("compliance.trans_z.k", default_lin_damp);
  // auto_declare<double>("compliance.rot_x.k", default_rot_damp);
  // auto_declare<double>("compliance.rot_y.k", default_rot_damp);
  // auto_declare<double>("compliance.rot_z.k", default_rot_damp);
  // // inertial
  // auto_declare<double>("compliance.trans_x.I", default_lin_inertial);
  // auto_declare<double>("compliance.trans_y.I", default_lin_inertial);
  // auto_declare<double>("compliance.trans_z.I", default_lin_inertial);
  // auto_declare<double>("compliance.rot_x.I", default_rot_inertial);
  // auto_declare<double>("compliance.rot_y.I", default_rot_inertial);
  // auto_declare<double>("compliance.rot_z.I", default_rot_inertial);


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

  // Make sure sensor wrenches are interpreted correctly
  // ForceBase::setFtSensorReferenceFrame(m_compliance_ref_link);

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

  const auto active_target = *m_target_frame_buffer.readFromRT();
  // Synchronize the internal model and the real robot
  Base::m_ik_solver->synchronizeJointPositions(Base::m_joint_state_pos_handles, period);

  // Control the robot motion in such a way that the resulting net force
  // vanishes. This internal control needs some simulation time steps.
  for (int i = 0; i < Base::m_iterations; ++i)
  {
    // The internal 'simulation time' is deliberately independent of the outer
    // control cycle.
    auto internal_period = rclcpp::Duration::from_seconds(0.005);

    // Compute the net force
    ctrl::Vector6D net_command = computeComplianceError(active_target);

    // Turn Cartesian error into joint motion
    Base::computeJointControlCmds(net_command, internal_period);
  }

  // Write final commands to the hardware interface
  Base::writeJointControlCmds();
  // additional publishing of motion error, wrench error, and wrench values for rosbag (position and twist is published through Base)
  MotionBase::publishMotionError(time);
  ForceBase::publishWrenches(time);


  return controller_interface::return_type::OK;
}

ctrl::Vector6D CartesianComplianceController::computeComplianceError(const KDL::Frame& active_target)
{
  // std::lock_guard<std::mutex> lock(m_param_mutex);

  MotionBase::computeMotionError(active_target);
  ctrl::Vector6D motion_command = Base::applyPDGains(MotionBase::m_gain_key, m_motion_error);

  // RCLCPP_INFO(get_node()->get_logger(), "spring force error: %f %f %f %f %f %f", net_force(0), net_force(1), net_force(2), net_force(3), net_force(4), net_force(5));
    // // Spring force in base orientation
    // Base::displayInBaseLink(m_stiffness, m_compliance_ref_link) * MotionBase::computeMotionError(target_frame)

  // Sensor and target force in base orientation
  ForceBase::computeForceError();
  ForceBase::updateContactState();
  ctrl::Vector6D force_command = Base::applyPDGains(ForceBase::m_gain_key, m_wrench_error, ForceBase::m_contact_state);

  ctrl::Vector6D net_force = motion_command + force_command;
  //RCLCPP_INFO(get_node()->get_logger(), "force error: %f %f %f %f %f %f", force_error(0), force_error(1), force_error(2), force_error(3), force_error(4), force_error(5));
  // RCLCPP_INFO(get_node()->get_logger(), "net force error: %f %f %f %f %f %f", net_force(0), net_force(1), net_force(2), net_force(3), net_force(4), net_force(5));

  // apply force deadband
  double f_threshold = 0.005; // N        based from simulation data noise tolerance 0.15-0.2N * K_pf gain (0.001)
  double t_threshold = 0.005; // Nm
  
  for (int i = 0; i < 6; ++i) {
    double limit = (i < 3) ? f_threshold : t_threshold;

    if (std::abs(net_force[i]) < limit) {
      net_force[i] = 0.0;
    } else {
      // deadband ramp
      net_force[i] -= std::copysign(limit, net_force[i]);
    }
  }

  return net_force;
}

// void CartesianComplianceController::calculateCriticalDamping(double zeta)
// {
//   for (int i = 0; i < 6; ++i)
//   {
//     // D = 2 * zeta * sqrt(M * K)
//     m_damping_diag[i] = 2.0 * zeta * std::sqrt(m_stiffness_diag[i] * m_inertia_diag[i]);
//   }
// }

}  // namespace cartesian_compliance_controller

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cartesian_compliance_controller::CartesianComplianceController,
                       controller_interface::ControllerInterface)
