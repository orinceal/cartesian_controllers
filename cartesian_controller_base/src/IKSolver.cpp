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
      m_ema_positions(i) = m_current_positions(i);
      m_last_positions(i) = m_current_positions(i);
      m_last_velocities(i) = m_current_velocities(i);
      m_filt_velocities(i) = m_current_velocities(i);
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

// when syncing with simulation, requires rate limiter and drift threshold to filter noise from physics solver
void IKSolver::synchronizeJointPositions(
  const std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> > &
    joint_pos_handles, const rclcpp::Duration & period)
{
  // M1 Alpha Blending
  // const double alpha = 0.1;
  const double pos_cutoff_hz = 50.0;
  const double tau = 1.0 / (2.0 * M_PI * pos_cutoff_hz);
  const double alpha = period.seconds() / (tau + period.seconds()); // at 200Hz with 10Hz cutoff, alpha = 0.005/(0.016 + 0.005) = 0.24

  // pull virtual model toward real (0 = no pull, 1 = instant snap)
  // at 200 Hz: 0.02 → ~10 cycle time constant
  const double sync_gain = 0.05;
  const double max_correction_rate = 0.05;  // rad/s — tune this
  const double max_correction = max_correction_rate * period.seconds();  // rad/cycle

  // M2 Snap within tolerance
  // const double tolerance = 0.0005;

  // M3
  // const double max_tracking_rate = 0.5;
  // const double max_delta = max_tracking_rate * period.seconds();

  for (size_t i = 0; i < joint_pos_handles.size(); ++i) {
    // Interface type should be checked by the caller.
    // Add additional plausibility check just in case.
    if (joint_pos_handles[i].get().get_interface_name() != hardware_interface::HW_IF_POSITION) continue;
    
    auto opt = joint_pos_handles[i].get().get_optional();
    if (!opt.has_value()) continue;
    double q_real = opt.value();

    // M0 direct sync

    // M1 EMA filter
    m_ema_positions(i) = (1.0 - alpha) * m_ema_positions(i) + alpha * q_real;
    //m_current_positions(i) = (1.0 - alpha) * m_current_positions(i) + alpha * q_real;

    // M2
    // only update if difference is outside the noise threshold
    // if (std::abs(q_real - m_current_positions(i)) > tolerance) {
    //   m_current_positions(i) = q_real;
    // }
    // m_last_positions(i) = m_current_positions(i);

    // M3 rate limiter on real positions to suppress physics noise
    // on real robot, this rate limit never activates
    // double error = q_real - m_ema_positions(i);
    // double max_delta = 0.1 * period.seconds(); // max rate = max_delta/dt = 0.0005/0.005 = 0.1 rad/s -> 0.0005 rad/cycle
    // clamp to max change per cycle
    // m_ema_positions(i) += std::clamp(error, -max_delta, max_delta);

    // // correct large drifts in virtual model to reduce velocity noise from integration
    // double virtual_drift = q_real - m_current_positions(i);
    // if (std::abs(virtual_drift) > 0.02) { //  drift threshold: 0.05rad = 50mrad 
    //   double correction = 0.02 * period.seconds(); 
    //   m_current_positions(i) += std::clamp(virtual_drift, -correction, correction);
    //   RCLCPP_WARN_THROTTLE(m_handle->get_logger(),*m_handle->get_clock(), 1000,
    //   "Joint %zu virtual drift %.4f rad", i, virtual_drift);
    // }

    // continuous proportional pull towards reality - no dead zone
    double virtual_drift = m_ema_positions(i) - m_current_positions(i);

    double correction = sync_gain * virtual_drift;
    m_current_positions(i) += std::clamp(correction, -max_correction, max_correction);
    if (std::abs(virtual_drift) > 0.05) {
      RCLCPP_WARN_THROTTLE(m_handle->get_logger(), *m_handle->get_clock(), 1000,
      "Joint %zu virtual drift %.4f rad", i, virtual_drift);
    }
  }
}

// For use with real hardware we can sync directly from encoders without filtering
// void IKSolver::synchronizeJointPositions(
//   const std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> > &
//     joint_pos_handles)
// {
//   for (size_t i = 0; i < joint_pos_handles.size(); ++i)
//   {
//     // Interface type should be checked by the caller.
//     // Add additional plausibility check just in case.
//     if (joint_pos_handles[i].get().get_interface_name() == hardware_interface::HW_IF_POSITION)
//     {
//       m_current_positions(i) = joint_pos_handles[i].get().get_value();
//       m_last_positions(i) = m_current_positions(i);
//     }
//   }
// }

bool IKSolver::init(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> nh, const KDL::Chain & chain,
                    const KDL::JntArray & upper_pos_limits, const KDL::JntArray & lower_pos_limits,
                    const KDL::JntArray & vel_limits, const KDL::JntArray & accel_limits)
{
  // Initialize
  m_handle = nh;
  m_chain = chain;
  m_number_joints = m_chain.getNrOfJoints();
  m_current_positions.data = ctrl::VectorND::Zero(m_number_joints);
  m_ema_positions.data = ctrl::VectorND::Zero(m_number_joints);
  m_current_velocities.data = ctrl::VectorND::Zero(m_number_joints);
  m_filt_velocities.data = ctrl::VectorND::Zero(m_number_joints);
  m_current_accelerations.data = ctrl::VectorND::Zero(m_number_joints);
  m_last_positions.data = ctrl::VectorND::Zero(m_number_joints);
  m_last_velocities.data = ctrl::VectorND::Zero(m_number_joints);
  m_ns_positions.data = ctrl::VectorND::Zero(m_number_joints);
  m_upper_pos_limits = upper_pos_limits;
  m_lower_pos_limits = lower_pos_limits;
  m_vel_limits = vel_limits;
  m_accel_limits = accel_limits;
  m_vel_limits_on   = nh->get_parameter("solver.velocity_limits_on").as_bool();
  m_vel_deadband_on = nh->get_parameter("solver.vel_deadband_on").as_bool();

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
  // m_fk_pos_solver->JntToCart(m_current_positions, m_end_effector_pose); // virtual positions
  m_fk_pos_solver->JntToCart(m_ema_positions, m_end_effector_pose); // real positions from hardware sync
  // apply filtering to current velocities
  filterVel();
  // Absolute velocity w. r. t. base
  KDL::FrameVel vel;
  // m_fk_vel_solver->JntToCart(KDL::JntArrayVel(m_current_positions, m_current_velocities), vel);
  m_fk_vel_solver->JntToCart(KDL::JntArrayVel(m_current_positions, m_filt_velocities), vel);
  m_end_effector_vel[0] = vel.deriv().vel.x();
  m_end_effector_vel[1] = vel.deriv().vel.y();
  m_end_effector_vel[2] = vel.deriv().vel.z();
  m_end_effector_vel[3] = vel.deriv().rot.x();
  m_end_effector_vel[4] = vel.deriv().rot.y();
  m_end_effector_vel[5] = vel.deriv().rot.z();
}

// void IKSolver::applyJointLimits()
// {
//   for (int i = 0; i < m_number_joints; ++i)
//   {
//     if (std::isnan(m_lower_pos_limits(i)) || std::isnan(m_upper_pos_limits(i)))
//     {
//       // Joint marked as continuous.
//       continue;
//     }
//     m_current_positions(i) =
//       std::clamp(m_current_positions(i), m_lower_pos_limits(i), m_upper_pos_limits(i));
//   }
// }

void IKSolver::applyJointLimits()
{
    for (int i = 0; i < m_number_joints; ++i) {
        if (std::isnan(m_lower_pos_limits(i)) ||
            std::isnan(m_upper_pos_limits(i))) continue;

        // For joint_4 — detect if flip is about to occur
        // by checking if position crossed the flip threshold
        // if (i == 1) {
        //     double q_prev = m_last_positions(i);
        //     double q_curr = m_current_positions(i);

        //     // Detect large jump indicating flip
        //     double dq = q_curr - q_prev;
        //     if (std::abs(dq) > 0.5) {  // >0.5 rad in one cycle = flip
        //         RCLCPP_WARN(m_handle->get_logger(),
        //             "Joint_4 flip detected! dq=%.3f — reverting", dq);
        //         // Revert to previous position
        //         m_current_positions(i) = q_prev;
        //         m_current_velocities(i) = 0.0;  // kill velocity
        //     }
        // }

        m_current_positions(i) = std::clamp(
            m_current_positions(i),
            m_lower_pos_limits(i),
            m_upper_pos_limits(i));
    }
}

void IKSolver::applyVelLimits()
{
  // Per-joint deadbands matching physical units
  const std::array<double, 7> joint_deadbands = {
      0.00005,   // Slider_18    (m/s)  — linear, lower deadband
      0.0005,   // robco_joint_0 (rad/s)
      0.0005,   // robco_joint_1
      0.0005,   // robco_joint_2
      0.0005,   // robco_joint_3
      0.0005,   // robco_joint_4 — lower stiffness, more noise
      0.001   // robco_joint_5 — lowest stiffness, most noise
  };

  // check if time to collision is decreasing
  // bool ttc_decreasing = (last_min_ttc_ - min_ttc_) > 0.01; // ttc noise

  for (int i = 0; i < m_number_joints; ++i)
  {
      double vel   = m_current_velocities(i);
      double limit = m_vel_limits(i);
      double db = (i < (int)joint_deadbands.size()) ? joint_deadbands[i] : m_vel_deadband; // fallback

      // Consider joint_0, joint_2 and joint_3, which rotation sweeps joint_4 motor through arc into wall
      // if (i == 1 || i == 3 || i == 4) {
      // if (ttc_decreasing && safety_factor_ > 0.0) {
      //   vel *= (1.0 - (safety_factor_ * 0.8)); // max 50% q_dot reduction per cycle
      //   limit = (1.0 - safety_factor_) * m_vel_limits(i);  // linear
      //   // limit = (1.0 - (safety_factor * safety_factor)); // quadratic
      //   // RCLCPP_WARN(m_handle->get_logger(),
      //   //     // *m_handle->get_clock(), 200,
      //   //     "Joint %d: min clearance: %.3fm, min ttc: %.3fs, limit: %.4f (safety_factor: %.4f)",
      //   //     i, collision_clearance_, min_ttc_, limit, safety_factor_);
      //   }
      // }

      // Deadband: zero out low velocities to prevent noise-driven drift
      if (m_vel_deadband_on) {
        if (std::abs(vel) < db) {
            vel = 0.0;
        }
        else {
            vel = (vel > 0) ? (vel - db)
                            : (vel + db);
        }
      }

      // Hard velocity clamp
      if (m_vel_limits_on) {
        vel = std::clamp(vel, -limit, limit);
      }
      m_current_velocities(i) = vel;
  }
  // last_min_ttc_ = min_ttc_;
}

// void IKSolver::applyVelLimits()
// {
//   for (int i = 0; i < m_number_joints; ++i)
//   {
//     double vel = m_current_velocities(i);
//     // apply deadband to prevent integral drift
//     if (std::abs(vel) < m_vel_deadband) {
//       vel = 0.0;
//     } else {
//       // subtract deadband to prevent jump when coming out of deadband
//       vel = (vel > 0) ? (vel - m_vel_deadband) : (vel + m_vel_deadband);
//     }
//     // apply hard limit clamps
//     m_current_velocities(i) = std::clamp(vel, -m_vel_limits(i), m_vel_limits(i));
//   }
// }

void IKSolver::filterVel()
{
  // filter velocities
  const double dt = 0.005; // controller period
  const double tau = 1.0/(2.0 * M_PI * m_vel_filter_cutoff);
  const double alpha = dt / (tau + dt);

  for (int i=0; i < m_number_joints; ++i) {
    m_filt_velocities(i) = (1.0 - alpha) * m_filt_velocities(i) + alpha * m_current_velocities(i);
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
