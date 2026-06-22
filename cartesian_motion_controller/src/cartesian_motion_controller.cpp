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
/*!\file    cartesian_motion_controller.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_motion_controller/cartesian_motion_controller.h>

#include <algorithm>
#include <cmath>

#include "cartesian_controller_base/Utility.h"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"

namespace cartesian_motion_controller
{
CartesianMotionController::CartesianMotionController() : Base::CartesianControllerBase() {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianMotionController::on_init()
{
  const auto ret = Base::on_init();
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianMotionController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  const auto ret = Base::on_configure(previous_state);
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // get node pointer (lock the weak ptr)
  auto node_ptr = get_node();
  if (!node_ptr) {
    RCLCPP_ERROR(get_node()->get_logger(), "Unable to lock node ptr in Motion Controller on_configure");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Initialize Cartesian pd controllers
  m_spatial_controller.init(node_ptr.get(), m_gain_key);

  // Create subscription
  // m_target_frame_subscriber = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
  //   get_node()->get_name() + std::string("/target_frame"), 3,
  //   std::bind(&CartesianMotionController::targetFrameCallback, this, std::placeholders::_1));
  m_target_frame_subscriber = node_ptr->create_subscription<geometry_msgs::msg::PoseStamped>(
    node_ptr->get_name() + std::string("/target_frame"), 3,
    std::bind(&CartesianMotionController::targetFrameCallback, this, std::placeholders::_1)
  );
  // Initialize realtime buffer
  m_target_frame_buffer.initRT(KDL::Frame());

  // Controller-internal state publishing
  m_target_pose_publisher = 
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
      node_ptr->create_publisher<geometry_msgs::msg::PoseStamped>(
        std::string(node_ptr->get_name()) + "/target_frame_filtered", 3));
  m_pos_error_raw_publisher = 
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::Vector3Stamped>>(
      node_ptr->create_publisher<geometry_msgs::msg::Vector3Stamped>(
        std::string(node_ptr->get_name()) + "/pos_error_raw", 3));
  m_rot_error_raw_publisher = 
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::Vector3Stamped>>(
      node_ptr->create_publisher<geometry_msgs::msg::Vector3Stamped>(
        std::string(node_ptr->get_name()) + "/rot_error_raw", 3));        
  m_pos_error_publisher = 
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::Vector3Stamped>>(
      node_ptr->create_publisher<geometry_msgs::msg::Vector3Stamped>(
        std::string(node_ptr->get_name()) + "/pos_error", 3));
  m_rot_error_publisher = 
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::Vector3Stamped>>(
      node_ptr->create_publisher<geometry_msgs::msg::Vector3Stamped>(
        std::string(node_ptr->get_name()) + "/rot_error", 3));        


  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianMotionController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  Base::on_activate(previous_state);

  // reset buffer and filter state to start where we are
  first_target_ = true;
  // m_error_filter_init_ = false;
  m_approach_vel_ = 0.0;
  m_target_frame_buffer.initRT(Base::m_ik_solver->getEndEffectorPose());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianMotionController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  Base::on_deactivate(previous_state);
  m_target_frame_subscriber.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianMotionController::update(const rclcpp::Time & time,
                                                                    const rclcpp::Duration & period)
{
  // KDL::Frame active_target;
  const auto active_target = *m_target_frame_buffer.readFromRT();
  // Synchronize the internal model and the real robot
  Base::m_ik_solver->synchronizeJointPositions(Base::m_joint_state_pos_handles, period);

  // Forward Dynamics turns the search for the according joint motion into a
  // control process. So, we control the internal model until we meet the
  // Cartesian target motion. This internal control needs some simulation time
  // steps.
  for (int i = 0; i < Base::m_iterations; ++i)
  {
    // The internal 'simulation time' is deliberately independent of the outer
    // control cycle.
    auto internal_period = rclcpp::Duration::from_seconds(0.005);
    // Compute the motion error = target - current.
    // ctrl::Vector6D error = computeMotionError(active_target);
    computeMotionError(active_target, internal_period);
    // apply PD gains: F_motion = K_p * (x_d - x) + D_p * (x_dot_d - x_dot)
    ctrl::Vector6D command = Base::applyPDGains(m_gain_key, m_motion_error);
    // Turn Cartesian error into joint motion    
    Base::computeJointControlCmds(command, internal_period);
  }

  // Write final commands to the hardware interface
  Base::writeJointControlCmds();

  return controller_interface::return_type::OK;
}

ctrl::Vector6D CartesianMotionController::computeMotionError(const KDL::Frame& target_frame, const rclcpp::Duration & period)
{
  // filter target
  KDL::Frame filtered_target = filterTarget(target_frame, period);

  // Compute motion error wrt robot_base_link
  const auto current_frame = Base::m_ik_solver->getEndEffectorPose();
  // Transformation from target -> current corresponds to error = target - current
  KDL::Vector pos_err = filtered_target.p - current_frame.p;
  KDL::Rotation rot_err = filtered_target.M * current_frame.M.Inverse();

  // Use Rodrigues Vector for a compact representation of orientation errors
  // Only for angles within [0,Pi)
  KDL::Vector rot_axis = KDL::Vector::Zero();
  double angle = rot_err.GetRotAngle(rot_axis);  // rot_axis is normalized

  // store raw errors for publishing before deadband/clamping
  m_target_frame = filtered_target;
  m_pos_error_raw = pos_err;
  m_rot_error_raw = rot_axis * angle; // rotation error (Rodrigues vector)

  // deadband parameters
  const double dist_deadband = 0.0001;      // 1mm (absolute zero)
  const double rot_deadband  = 0.002; // 0.035;       // ~2 degrees

  // Clamp maximal tolerated error.
  // The remaining error will be handled in the next control cycle.
  // Note that this is also the maximal offset that the
  // cartesian_compliance_controller can use to build up a restoring stiffness
  // wrench.
  const double max_distance = 0.1;
  const double max_angle = 0.1;

  double net_pos_error = pos_err.Norm();

  // apply spherical deadband to linear error
  if (net_pos_error <= dist_deadband) {
    m_motion_error[0] = 0.0;
    m_motion_error[1] = 0.0;
    m_motion_error[2] = 0.0;
  } else { 
    KDL::Vector direction = pos_err;
    direction.Normalize();

    double shifted_magnitude = net_pos_error - dist_deadband;
    double clamped_magnitude = std::clamp(shifted_magnitude, 0.0, max_distance);
    KDL::Vector final_pos_err = direction * clamped_magnitude;
    m_motion_error[0] = final_pos_err(0);
    m_motion_error[1] = final_pos_err(1);
    m_motion_error[2] = final_pos_err(2);
  }
  m_pos_error(0) = m_motion_error[0];
  m_pos_error(1) = m_motion_error[1];
  m_pos_error(2) = m_motion_error[2];

  // apply spherical deadband to rotational error
  double net_rot_error = std::abs(angle);

  if (net_rot_error <= rot_deadband) {
    m_motion_error[3] = 0.0;
    m_motion_error[4] = 0.0;
    m_motion_error[5] = 0.0;
  } else {
    double shifted_angle = net_rot_error - rot_deadband;
    double clamped_angle = std::clamp(shifted_angle, 0.0, max_angle);

    KDL::Vector final_rot_err = rot_axis * clamped_angle;
    m_motion_error[3] = final_rot_err(0);
    m_motion_error[4] = final_rot_err(1);
    m_motion_error[5] = final_rot_err(2);
  }  
  m_rot_error(0) = m_motion_error[3];
  m_rot_error(1) = m_motion_error[4];
  m_rot_error(2) = m_motion_error[5];


  return m_motion_error;
}

void CartesianMotionController::targetFrameCallback(const geometry_msgs::msg::PoseStamped::SharedPtr target)
{
  if (!this->isActive())
  {
    return;
  }

  if (std::isnan(target->pose.position.x) || std::isnan(target->pose.position.y) ||
      std::isnan(target->pose.position.z) || std::isnan(target->pose.orientation.x) ||
      std::isnan(target->pose.orientation.y) || std::isnan(target->pose.orientation.z) ||
      std::isnan(target->pose.orientation.w))
  {
    auto & clock = *get_node()->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(), clock, 3000,
                                "NaN detected in target pose. Ignoring input.");
    return;
  }

  if (target->header.frame_id != Base::m_robot_base_link)
  {
    auto & clock = *get_node()->get_clock();
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), clock, 3000,
                         "Got target pose in wrong reference frame. Expected: %s but got %s",
                         Base::m_robot_base_link.c_str(), target->header.frame_id.c_str());
    return;
  }

  KDL::Frame target_raw(
    KDL::Rotation::Quaternion(target->pose.orientation.x, target->pose.orientation.y,
                                target->pose.orientation.z, target->pose.orientation.w),
    KDL::Vector(target->pose.position.x, target->pose.position.y, target->pose.position.z));
  
  // KDL::Frame smooth_target = filterTarget(target_raw);
  m_target_frame_buffer.writeFromNonRT(target_raw); // non-blocking write    
}

KDL::Frame CartesianMotionController::filterTarget(const KDL::Frame & target_raw, const rclcpp::Duration & period) {
  if (first_target_) {
    filtered_target_frame_ = target_raw;
    first_target_ = false;
    return filtered_target_frame_;
  }

  // double alpha = 0.1;
  // // apply linear interpolation to position
  // filtered_target_frame_.p = (1.0 - alpha) * filtered_target_frame_.p + alpha * target_raw.p;

  // // apply SLERP to quarternions 
  // KDL::Vector rot_diff = KDL::diff(filtered_target_frame_.M, target_raw.M); // rotation vector from A to B
  // double angle = rot_diff.Norm();

  double dt = period.seconds();

  // max approach rates
  const double max_vel = 0.2; // m/s
  const double max_accel = 0.3; // m/s2
  // const double max_angular_vel = 0.15; // rad/s

  // linear: move at constant rate toward target (ramp profile)
  KDL::Vector dp = target_raw.p - filtered_target_frame_.p;
  double dist = dp.Norm();

  if (dist < 1e-6) {
    m_approach_vel_ = 0.0;
  } else {
    // braking distance at current speed: v^2 / 2a
    double brake_dist = (m_approach_vel_ * m_approach_vel_) / (2.0 * max_accel);
    // reduce target speed if within braking distance, else accelerate
    double target_vel = (dist <= brake_dist + 1e-4) ? std::sqrt(2.0 * max_accel * dist) : max_vel;

    // rate limit the speed change itself
    double dv = std::clamp(target_vel - m_approach_vel_, -max_accel * dt, max_accel * dt);
    m_approach_vel_ = std::max(0.0, m_approach_vel_ + dv);

    double step = m_approach_vel_ * dt;
    filtered_target_frame_.p += dp * std::min(1.0, step / dist);
  }
  // rotation: move at constant angular rate toward target
  // KDL::Vector rot_diff = KDL::diff(filtered_target_frame_.M, target_raw.M);
  // double angle = rot_diff.Norm();

  // if (angle > 1e-6) { // only rotate if there is a meaningful difference
  //   KDL::Vector axis = rot_diff / angle;
  //   // KDL::Rotation rot_inc = KDL::Rotation::Rot2(axis, angle * alpha);
  //   // filtered_target_frame_.M = filtered_target_frame_.M * rot_inc;

  //   // renormalize to prevent S0(3) drift (apply over 100 runs)
  //   // double x, y, z, w;
  //   // filtered_target_frame_.M.GetQuaternion(x, y, z, w);
  //   // double norm = std::sqrt(x*x + y*y + z*z + w*w);
  //   // x /= norm; y /= norm; z /= norm; w /= norm;
  //   // filtered_target_frame_.M = KDL::Rotation::Quaternion(x, y, z, w);

  //   double applied_angle = std::min(angle, max_angular_vel * dt);
  //   filtered_target_frame_.M = filtered_target_frame_.M * KDL::Rotation::Rot2(axis, applied_angle);
  // }
  filtered_target_frame_.M = target_raw.M;
  return filtered_target_frame_;
}

void CartesianMotionController::publishMotionError(const rclcpp::Time& time) {
    if (m_pos_error_raw_publisher && m_pos_error_raw_publisher->trylock()){
      auto& msg = m_pos_error_raw_publisher->msg_;
      msg.header.stamp = time;
      msg.header.frame_id = Base::m_robot_base_link;
      msg.vector.x  = m_pos_error_raw(0);
      msg.vector.y  = m_pos_error_raw(1);
      msg.vector.z  = m_pos_error_raw(2);
      m_pos_error_raw_publisher->unlockAndPublish();
    } 
    if (m_rot_error_raw_publisher && m_rot_error_raw_publisher->trylock()){
      // double roll, pitch, yaw;
      // m_rot_error_raw.GetRPY(roll, pitch, yaw);

      auto& msg = m_rot_error_raw_publisher->msg_;
      msg.header.stamp = time;
      msg.header.frame_id = Base::m_robot_base_link;
      // msg.vector.x = roll;
      // msg.vector.y = pitch;
      // msg.vector.z = yaw;
      msg.vector.x = m_rot_error_raw(0);
      msg.vector.y = m_rot_error_raw(1);
      msg.vector.z = m_rot_error_raw(2);
      m_rot_error_raw_publisher->unlockAndPublish();
    }
    if (m_pos_error_publisher && m_pos_error_publisher->trylock()){
      auto& msg = m_pos_error_publisher->msg_;
      msg.header.stamp = time;
      msg.header.frame_id = Base::m_robot_base_link;
      msg.vector.x  = m_pos_error(0);
      msg.vector.y  = m_pos_error(1);
      msg.vector.z  = m_pos_error(2);
      m_pos_error_publisher->unlockAndPublish();
    } 
    if (m_rot_error_publisher && m_rot_error_publisher->trylock()){
      auto& msg = m_rot_error_publisher->msg_;
      msg.header.stamp = time;
      msg.header.frame_id = Base::m_robot_base_link;
      msg.vector.x = m_rot_error(0);
      msg.vector.y = m_rot_error(1);
      msg.vector.z = m_rot_error(2);
      m_rot_error_publisher->unlockAndPublish();
    }
    // if (m_target_pose_publisher && m_target_pose_publisher->trylock()){
    //   auto& msg = m_target_pose_publisher->msg_;
    //   msg.header.stamp = time;
    //   msg.header.frame_id = Base::m_robot_base_link;

    //   msg.pose.position.x = m_target_frame.p.x();
    //   msg.pose.position.y = m_target_frame.p.y();
    //   msg.pose.position.z = m_target_frame.p.z();

    //   double x, y, z, w;
    //   m_target_frame.M.GetQuaternion(x, y, z, w);
    //   msg.pose.orientation.x = x;
    //   msg.pose.orientation.y = y;
    //   msg.pose.orientation.z = z;
    //   msg.pose.orientation.w = w;
    //   m_target_pose_publisher->unlockAndPublish();
    // }    
  }
}  // namespace cartesian_motion_controller

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cartesian_motion_controller::CartesianMotionController,
                       controller_interface::ControllerInterface)
