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
/*!\file    IKSolver.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2016/02/14
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_controller_base/IKSolver.h>

#include <algorithm>
#include <functional>
#include <kdl/framevel.hpp>
#include <kdl/jntarrayvel.hpp>
#include <map>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/node.hpp"

namespace cartesian_controller_base
{
IKSolver::IKSolver() {}

IKSolver::~IKSolver() {}

const KDL::Frame & IKSolver::getEndEffectorPose() const { return m_end_effector_pose; }

const ctrl::Vector6D & IKSolver::getEndEffectorVel() const { return m_end_effector_vel; }

const KDL::JntArray & IKSolver::getPositions() const { return m_current_positions; }

bool IKSolver::setStartState(
  const std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> > &
    joint_pos_handles)
{
  // Copy into internal buffers.
  for (size_t i = 0; i < joint_pos_handles.size(); ++i)
  {
    // Interface type should be checked by the caller.
    // Add additional plausibility check just in case.
    if (joint_pos_handles[i].get().get_interface_name() == hardware_interface::HW_IF_POSITION)
    {
      auto opt_value = joint_pos_handles[i].get().get_optional();
      if (opt_value.has_value()){
        m_current_positions(i) = opt_value.value();
      }
      m_current_velocities(i) = 0.0;
      m_current_accelerations(i) = 0.0;
      m_last_positions(i) = m_current_positions(i);
      m_last_velocities(i) = m_current_velocities(i);
      // initialize beginning safe positions
      m_ns_positions(i) = m_current_positions(i);
    }
    else
    {
      return false;
    }
  }
  return true;
}

void IKSolver::synchronizeJointPositions(
  const std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> > &
    joint_pos_handles, const rclcpp::Duration & period)
{
  // M1 Alpha Blending
  const double alpha = 0.1;
  static bool first_sync = true;

  // M2 Snap within tolerance
  // const double tolerance = 0.0005;

  // M3
  const double max_tracking_rate = 0.5;
  const double max_delta = max_tracking_rate * period.seconds();

  for (size_t i = 0; i < joint_pos_handles.size(); ++i)
  {
    // Interface type should be checked by the caller.
    // Add additional plausibility check just in case.
    if (joint_pos_handles[i].get().get_interface_name() == hardware_interface::HW_IF_POSITION)
    {
      auto opt_value = joint_pos_handles[i].get().get_optional();
      if (opt_value.has_value()){
        double q_real = opt_value.value();
      
        // M1
        if (first_sync) {
          m_current_positions(i) = q_real;
        } else {
        // blend new reading with existing internal state
        //m_current_positions(i) = (1.0 - alpha) * m_current_positions(i) + alpha * q_real;

        // M2
        // only update if difference is outside the noise threshold
        // if (std::abs(q_real - m_current_positions(i)) > tolerance) {
        //   m_current_positions(i) = q_real;
        // }
        // m_last_positions(i) = m_current_positions(i);

        // M3
        double error = q_real - m_current_positions(i);
        // clamp to max change per cycle
        double clamped = std::clamp(error, -max_delta, max_delta);
        m_current_positions(i) += clamped;
        }
      }
    }
  }
  first_sync = false;
}

bool IKSolver::init(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> nh, const KDL::Chain & chain,
                    const KDL::JntArray & upper_pos_limits, const KDL::JntArray & lower_pos_limits,
                    const KDL::JntArray & vel_limits, const KDL::JntArray & accel_limits)
{
  // Initialize
  m_handle = nh;
  m_chain = chain;
  m_number_joints = m_chain.getNrOfJoints();
  m_current_positions.data = ctrl::VectorND::Zero(m_number_joints);
  m_current_velocities.data = ctrl::VectorND::Zero(m_number_joints);
  m_current_accelerations.data = ctrl::VectorND::Zero(m_number_joints);
  m_last_positions.data = ctrl::VectorND::Zero(m_number_joints);
  m_last_velocities.data = ctrl::VectorND::Zero(m_number_joints);
  m_ns_positions.data = ctrl::VectorND::Zero(m_number_joints);
  m_upper_pos_limits = upper_pos_limits;
  m_lower_pos_limits = lower_pos_limits;
  m_vel_limits = vel_limits;
  m_accel_limits = accel_limits;

  // Forward kinematics
  m_fk_pos_solver = std::make_shared<KDL::ChainFkSolverPos_recursive>(m_chain);
  m_fk_vel_solver = std::make_shared<KDL::ChainFkSolverVel_recursive>(m_chain);
  // m_fk_pos_solver.reset(new KDL::ChainFkSolverPos_recursive(m_chain));
  // m_fk_vel_solver.reset(new KDL::ChainFkSolverVel_recursive(m_chain));

  return true;
}

void IKSolver::updateKinematics()
{
  // Pose w. r. t. base
  m_fk_pos_solver->JntToCart(m_current_positions, m_end_effector_pose);

  // Absolute velocity w. r. t. base
  KDL::FrameVel vel;
  m_fk_vel_solver->JntToCart(KDL::JntArrayVel(m_current_positions, m_current_velocities), vel);
  m_end_effector_vel[0] = vel.deriv().vel.x();
  m_end_effector_vel[1] = vel.deriv().vel.y();
  m_end_effector_vel[2] = vel.deriv().vel.z();
  m_end_effector_vel[3] = vel.deriv().rot.x();
  m_end_effector_vel[4] = vel.deriv().rot.y();
  m_end_effector_vel[5] = vel.deriv().rot.z();
}

void IKSolver::applyJointLimits()
{
  for (int i = 0; i < m_number_joints; ++i)
  {
    if (std::isnan(m_lower_pos_limits(i)) || std::isnan(m_upper_pos_limits(i)))
    {
      // Joint marked as continuous.
      continue;
    }
    m_current_positions(i) =
      std::clamp(m_current_positions(i), m_lower_pos_limits(i), m_upper_pos_limits(i));
  }
}

void IKSolver::applyVelLimits()
{
  for (int i = 0; i < m_number_joints; ++i)
  {
    double vel = m_current_velocities(i);
    // apply deadband to prevent integral drift
    if (std::abs(vel) < m_vel_deadband) {
      vel = 0.0;
    } else {
      // subtract deadband to prevent jump when coming out of deadband
      vel = (vel > 0) ? (vel - m_vel_deadband) : (vel + m_vel_deadband);
    }
    // apply limit clamps
    m_current_velocities(i) = std::clamp(vel, -m_vel_limits(i), m_vel_limits(i));
  }
}

void IKSolver::applyAccelLimits()
{
  for (int i = 0; i < m_number_joints; ++i)
  {
    m_current_accelerations(i) = 
      std::clamp(m_current_accelerations(i), -m_accel_limits(i), m_accel_limits(i));
    // if (std::abs(m_current_accelerations(i)) < m_accel_deadband) {
    //   m_current_accelerations(i) = 0.0;
    // }
  }
}
}  // namespace cartesian_controller_base
