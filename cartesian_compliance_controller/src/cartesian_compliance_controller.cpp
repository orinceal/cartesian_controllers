////////////////////////////////////////////////////////////////////////////////
// Copyright 2026 Sitegeist GmbH
//
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
 * \author  Jolene Ng <jolene.ng@tum.de>
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2026/07/15
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

  // get node pointer (lock the weak ptr)
  auto node_ptr = get_node();
  if (!node_ptr) {
    RCLCPP_ERROR(get_node()->get_logger(), "Unable to lock node ptr in Compliance Controller on_configure");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  // intialize gains for constrained motion during contact
  m_spatial_controller.init(node_ptr.get(), m_contact_motion_gain_key);

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

  const auto target_frame = *m_target_frame_buffer.readFromRT();
  const auto target_lin_vel = *m_target_vel_buffer.readFromRT();

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
    ctrl::Vector6D net_command = computeComplianceError(target_frame, target_lin_vel, internal_period);

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

ctrl::Vector6D CartesianComplianceController::computeComplianceError(const KDL::Frame& target_frame, const Eigen::Vector3d target_lin_vel, const rclcpp::Duration & period)
{
  ForceBase::computeForceError();
  ctrl::Vector6D force_damping_term = Base::m_ik_solver->getEndEffectorVel();
  ForceBase::updateContactState();
  ctrl::Vector6D force_command = Base::applyPDGains(ForceBase::m_gain_key, m_wrench_error, force_damping_term, ForceBase::m_contact_state);

  std::string motion_key;
  if (ForceBase::m_contact_state == cartesian_controller_base::ContactState::FREE)
  {
    motion_key = MotionBase::m_gain_key; // use free motion gains when not in contact
  } else {
    motion_key = m_contact_motion_gain_key;
  }

  MotionBase::computeMotionError(target_frame, period);
  ctrl::Vector6D motion_damping_term = MotionBase::computeMotionDampingRef(target_lin_vel);
  ctrl::Vector6D motion_command = Base::applyPDGains(motion_key, m_motion_error, motion_damping_term);

  ctrl::Vector6D net_force = motion_command + force_command;

  // apply force deadband
  double f_threshold = 0.001; // N        based from simulation data noise tolerance 0.15-0.2N * K_pf gain (0.001) positional error y 0.000138 * K_p gain (3.0) = 0.0045
  double t_threshold = 0.002; // Nm      0.00038 * 4.0
  
  // // axis-by-axis deadband
  // for (int i = 0; i < 6; ++i) {
  //   double limit = (i < 3) ? f_threshold : t_threshold;

  //   if (std::abs(net_force[i]) < limit) {
  //     net_force[i] = 0.0;
  //   } else {
  //     // deadband ramp
  //     net_force[i] -= std::copysign(limit, net_force[i]);
  //   }
  // }

  // net magnitude deadbands
  double f_mag = std::sqrt(net_force[0]*net_force[0] + net_force[1]*net_force[1] + net_force[2]*net_force[2]);
  double t_mag = std::sqrt(net_force[3]*net_force[3] + net_force[4]*net_force[4] + net_force[5]*net_force[5]);

  if (f_mag < f_threshold) {
    net_force[0] = net_force[1] = net_force[2] = 0.0;
  } else {
    // scale by same deadband ratio
    double scale = (f_mag - f_threshold) / f_mag;
    net_force[0] *= scale;
    net_force[1] *= scale;
    net_force[2] *= scale;
  }
  if (t_mag < t_threshold) {
    net_force[3] = net_force[4] = net_force[5] = 0.0;
  } else { 
    double scale = (t_mag - t_threshold) / t_mag;
    net_force[3] *= scale;
    net_force[4] *= scale;
    net_force[5] *= scale;
  }

  return net_force;
}

}  // namespace cartesian_compliance_controller

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cartesian_compliance_controller::CartesianComplianceController,
                       controller_interface::ControllerInterface)
