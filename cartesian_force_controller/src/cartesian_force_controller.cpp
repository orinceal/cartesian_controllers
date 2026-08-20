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
/*!\file    cartesian_force_controller.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_force_controller/cartesian_force_controller.h>

#include <cmath>

#include "cartesian_controller_base/Utility.h"
#include "controller_interface/controller_interface.hpp"

namespace cartesian_force_controller
{
using cartesian_controller_base::ContactState;

CartesianForceController::CartesianForceController()
: Base::CartesianControllerBase(), m_hand_frame_control(false)
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianForceController::on_init()
{
  const auto ret = Base::on_init();
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  auto_declare<std::string>("ft_sensor_ref_link", "");
  auto_declare<bool>("hand_frame_control", false);
  auto_declare<std::string>("ft_sensor_topic", "isaac_sensor_wrench_filtered");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianForceController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  const auto ret = Base::on_configure(previous_state);
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // Make sure sensor link is part of the robot chain
  m_ft_sensor_ref_link = get_node()->get_parameter("ft_sensor_ref_link").as_string();
  if (!Base::robotChainContains(m_ft_sensor_ref_link))
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), m_ft_sensor_ref_link
                                                    << " is not part of the kinematic chain from "
                                                    << Base::m_robot_base_link << " to "
                                                    << Base::m_end_effector_link);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Define transformation that applies to sensor wrenches
  setFtSensorReferenceFrame(Base::m_end_effector_link);

  // get node pointer (lock the weak ptr)
  auto node_ptr = get_node();
  if (!node_ptr) {
    RCLCPP_ERROR(get_node()->get_logger(), "Unable to lock node ptr in Force Controller on_configure");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Initialize Cartesian pd controllers
  m_spatial_controller.init(node_ptr.get(), m_gain_key);

  // Create subscriptions
  m_target_wrench_subscriber = node_ptr->create_subscription<geometry_msgs::msg::WrenchStamped>(
    node_ptr->get_name() + std::string("/target_wrench"), 10,
    std::bind(&CartesianForceController::targetWrenchCallback, this, std::placeholders::_1));

  std::string ft_topic = get_node()->get_parameter("ft_sensor_topic").as_string();
  m_ft_sensor_wrench_subscriber = node_ptr->create_subscription<geometry_msgs::msg::WrenchStamped>(
      std::string("~/") + ft_topic, 10,
      std::bind(&CartesianForceController::ftSensorWrenchCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_node()->get_logger(), 
    "Subscribing to FT sensor topic: ~/%s", ft_topic.c_str());      

  // Initialize realtime buffers and parametes
  m_target_wrench_buffer.initRT(KDL::Wrench());
  m_ft_sensor_wrench_buffer.initRT(KDL::Wrench());
  // m_target_wrench.setZero();
  // m_ft_sensor_wrench.setZero();
  m_hand_frame_control = get_node()->get_parameter("hand_frame_control").as_bool();
  m_contact_state = ContactState::FREE;

  // Controller-internal state publishing
  m_target_wrench_pub = 
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>(
      node_ptr->create_publisher<geometry_msgs::msg::WrenchStamped>(
        std::string(node_ptr->get_name()) + "/target_wrench_base", 3));
  m_sensor_wrench_base_pub = 
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>(
      node_ptr->create_publisher<geometry_msgs::msg::WrenchStamped>(
        std::string(node_ptr->get_name()) + "/sensor_wrench_base", 3));
  m_wrench_error_raw_pub = 
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>(
      node_ptr->create_publisher<geometry_msgs::msg::WrenchStamped>(
        std::string(node_ptr->get_name()) + "/wrench_error_raw", 3));        
  m_wrench_error_pub = 
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>(
      node_ptr->create_publisher<geometry_msgs::msg::WrenchStamped>(
        std::string(node_ptr->get_name()) + "/wrench_error", 3));        


  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianForceController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  Base::on_activate(previous_state);
  // reset buffers
  m_target_wrench_buffer.initRT(KDL::Wrench());
  m_ft_sensor_wrench_buffer.initRT(KDL::Wrench());
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianForceController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  Base::on_deactivate(previous_state);
  m_target_wrench_subscriber.reset();
  m_ft_sensor_wrench_subscriber.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianForceController::update(const rclcpp::Time & time,
                                                                   const rclcpp::Duration & period)
{
  // Synchronize the internal model and the real robot
  Base::m_ik_solver->synchronizeJointPositions(Base::m_joint_state_pos_handles, period);

  // Control the robot motion in such a way that the resulting net force
  // vanishes.  The internal 'simulation time' is deliberately independent of
  // the outer control cycle.
  auto internal_period = rclcpp::Duration::from_seconds(0.02);
  // Compute the net force
  computeForceError();
  updateContactState();
  // get current x_dot as damping term
  ctrl::Vector6D x_dot = Base::m_ik_solver->getEndEffectorVel();

  // apply PD gains: F_force = K_p * (f_d - f) - D_f * x_dot
  ctrl::Vector6D command = Base::applyPDGains(m_gain_key, m_wrench_error, x_dot);
  // Turn Cartesian error into joint motion
  Base::computeJointControlCmds(command, internal_period);
  // Write final commands to the hardware interface
  Base::writeJointControlCmds();

  return controller_interface::return_type::OK;
}

ctrl::Vector6D CartesianForceController::computeForceError()
{
  // compute rotation only every N cycles to reduce computational load
  if (m_transform_update_counter ++ >= m_transform_update_cycle) {
    m_transform_update_counter = 0;
    m_wrench_base_rot = Base::rotationToBase(m_new_ft_sensor_ref);
  }  
  const auto target_wrench = *m_target_wrench_buffer.readFromRT();
  const auto current_wrench = *m_ft_sensor_wrench_buffer.readFromRT();

  if (m_hand_frame_control) { // subscribed wrench is commanded in end-effector frame
    m_target_wrench_base = m_wrench_base_rot * target_wrench; // assume target_wrench is given in the same frame as the transformed wrench reading
  } else { // subscribed wrench is already in base frame
    m_target_wrench_base = target_wrench;
  }
  // m_sensor_wrench_base = Base::displayInBaseLink(current_wrench, m_new_ft_sensor_ref);
  m_sensor_wrench_base = m_wrench_base_rot * current_wrench; 
  // Superimpose target wrench and sensor wrench in base frame
  m_wrench_error_kdl_raw = m_sensor_wrench_base + m_target_wrench_base;

  // apply deadband
  const double force_deadband = 0.2; // N
  const double torque_deadband = 0.05; // Nm

  for (int i=0; i < 6; ++i){
    double threshold = (i < 3) ? force_deadband : torque_deadband;
    if (std::abs(m_wrench_error_kdl_raw(i)) < threshold) {
      m_wrench_error_kdl(i) = 0.0;
    } else {
      m_wrench_error_kdl(i) = m_wrench_error_kdl_raw(i) - std::copysign(threshold, m_wrench_error_kdl_raw(i));
    }
    m_wrench_error[i] = m_wrench_error_kdl(i);
  }
  return m_wrench_error;
}

void CartesianForceController::setFtSensorReferenceFrame(const std::string & new_ref)
{
  // Compute static transform from the force torque sensor to the new reference
  // frame of interest.
  m_new_ft_sensor_ref = new_ref;

  // Joint positions should cancel out, i.e. it doesn't matter as long as they
  // are the same for both transformations.
  KDL::JntArray jnts(Base::m_ik_solver->getPositions());

  KDL::Frame sensor_ref;
  Base::m_forward_kinematics_solver->JntToCart(jnts, sensor_ref, m_ft_sensor_ref_link);

  KDL::Frame new_sensor_ref;
  Base::m_forward_kinematics_solver->JntToCart(jnts, new_sensor_ref, m_new_ft_sensor_ref);

  // set transformation from sensor to new_ref
  m_ft_sensor_transform = new_sensor_ref.Inverse() * sensor_ref;
}

void CartesianForceController::targetWrenchCallback(
  const geometry_msgs::msg::WrenchStamped::SharedPtr wrench)
{
  if (!this->isActive())
  {
    return;
  }

  if (std::isnan(wrench->wrench.force.x) || std::isnan(wrench->wrench.force.y) ||
      std::isnan(wrench->wrench.force.z) || std::isnan(wrench->wrench.torque.x) ||
      std::isnan(wrench->wrench.torque.y) || std::isnan(wrench->wrench.torque.z))
  {
    auto & clock = *get_node()->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(), clock, 3000,
                                "NaN detected in target wrench. Ignoring input.");
    return;
  }
  KDL::Wrench target_wrench;
  target_wrench[0] = wrench->wrench.force.x;
  target_wrench[1] = wrench->wrench.force.y;
  target_wrench[2] = wrench->wrench.force.z;
  target_wrench[3] = wrench->wrench.torque.x;
  target_wrench[4] = wrench->wrench.torque.y;
  target_wrench[5] = wrench->wrench.torque.z;
  m_target_wrench_buffer.writeFromNonRT(target_wrench);
}

void CartesianForceController::ftSensorWrenchCallback(
  const geometry_msgs::msg::WrenchStamped::SharedPtr wrench)
{
  if (!this->isActive())
  {
    return;
  }

  if (std::isnan(wrench->wrench.force.x) || std::isnan(wrench->wrench.force.y) ||
      std::isnan(wrench->wrench.force.z) || std::isnan(wrench->wrench.torque.x) ||
      std::isnan(wrench->wrench.torque.y) || std::isnan(wrench->wrench.torque.z))
  {
    auto & clock = *get_node()->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(), clock, 3000,
                                "NaN detected in force-torque sensor wrench. Ignoring input.");
    return;
  }

  KDL::Wrench tmp;
  tmp[0] = wrench->wrench.force.x;
  tmp[1] = wrench->wrench.force.y;
  tmp[2] = wrench->wrench.force.z;
  tmp[3] = wrench->wrench.torque.x;
  tmp[4] = wrench->wrench.torque.y;
  tmp[5] = wrench->wrench.torque.z;

  // Compute how the measured wrench appears in the frame of interest.
  tmp = m_ft_sensor_transform * tmp;
  // RCLCPP_INFO(get_node()->get_logger(), "sensor_wrench transformed: %f %f %f %f %f %f", tmp(0), tmp(1), tmp(2), tmp(3), tmp(4), tmp(5));

  // TODO: m_gravity_compensated_wrench = transformed_wrench - tool_gravity_wrench; // subtract weight of tool after transformation not before. but apply tare before transformation!
  m_ft_sensor_wrench_buffer.writeFromNonRT(tmp);
}

void CartesianForceController::updateContactState() {
  // get current force into wall
  double force_magnitude = m_sensor_wrench_base.force.Norm();
  double force_target = m_target_wrench_base.force.Norm();
  rclcpp::Time now = get_node()->get_clock()->now();

  // use fixed min threshold when target force is small/zero
  const double min_contact_threshold = 8.0;
  double contact_threshold = std::max(force_target * 1.25, min_contact_threshold);
  double release_threshold = std::max(force_target * 0.2, 1.0);

  // force target boundaries 
  double target_upper = force_target * 1.3;
  double target_lower = force_target * 0.7;
  bool force_check = (force_magnitude >= target_lower) && (force_magnitude <= target_upper);

  switch (m_contact_state) {
    case ContactState::FREE:
      if (force_magnitude >= contact_threshold) {
          // impact detected, switch state immediately
          m_contact_state = ContactState::IMPACT;
          m_impact_timer = now;
          RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
            "Contact established: %.3fN (threshold: %.3fN). Additional force damping enabled", 
            force_magnitude, contact_threshold);
      }
      break;
      
    case ContactState::CONTACT:
      // switch to impact state if force spikes
      // if (force_magnitude > force_target * 1.7) {
      //   m_contact_state = ContactState::IMPACT;
      //   m_impact_timer = now;
      //   RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
      //     "Force spike: %.3fN. Additional damping for impact enabled", force_magnitude);
      // }
      // contactCheck(force_magnitude, release_threshold, now);
      break;

    case ContactState::IMPACT:
    {
      // switch to contact mode when force settles or after timeout duration
      double impact_duration = (now - m_impact_timer).seconds();
      bool impact_timeout = impact_duration > m_max_impact_duration;

      if (impact_timeout) {
        m_contact_state = ContactState::CONTACT;
        m_settle_timer_running = false;
        m_release_timer_running = false;
        RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
          "Impact timeout: %.3fs. Current force: %.3fN  Normal force damping enabled", 
          impact_duration, force_magnitude);
      } else if (force_check) {
        // force within limits, start or check settle timer
        if (!m_settle_timer_running) {
          m_settle_timer = now;
          m_settle_timer_running = true;
        } else if ((now - m_settle_timer).seconds() > m_min_settle_duration) {
          // sustained force state, transition to CONTACT
          m_contact_state = ContactState::CONTACT;
          m_settle_timer_running = false;
          m_release_timer_running = false;
          RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
            "Stable contact established: %.3fN in %.3fs. Normal force damping enabled", 
            force_magnitude, impact_duration);
        }
      } else {
        m_settle_timer_running = false;
      }
      // contactCheck(force_magnitude, release_threshold, now);
      break;
    }
  }
}

void CartesianForceController::contactCheck(double force_magnitude, double release_threshold, const rclcpp::Time& now) {
  // use hysteresis, lower threshold to release than to engage
  if (force_magnitude < release_threshold) {
    if (!m_release_timer_running) {
      m_release_timer = now;
      m_release_timer_running = true;
    } else if ((now - m_release_timer).seconds() > m_release_contact_duration) {
      m_contact_state = ContactState::FREE;
      m_release_timer_running = false;
      RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500, "Contact lost: %.3fN. Force damping disabled", force_magnitude);
    }
  } else {
    m_release_timer_running = false;
  }
}

void CartesianForceController::publishWrenches(const rclcpp::Time& time) {
  if (m_target_wrench_pub && m_target_wrench_pub->trylock()){
    auto& msg = m_target_wrench_pub->msg_;
    msg.header.stamp = time;
    msg.header.frame_id = Base::m_robot_base_link;
    msg.wrench = KDLWrenchToWrenchMsg(m_target_wrench_base);
    m_target_wrench_pub->unlockAndPublish();
  }
  if (m_sensor_wrench_base_pub && m_sensor_wrench_base_pub->trylock()){
    auto& msg = m_sensor_wrench_base_pub->msg_;
    msg.header.stamp = time;
    msg.header.frame_id = Base::m_robot_base_link;
    msg.wrench = KDLWrenchToWrenchMsg(m_sensor_wrench_base);
    m_sensor_wrench_base_pub->unlockAndPublish();
  }
  if (m_wrench_error_raw_pub && m_wrench_error_raw_pub->trylock()){
    auto& msg = m_wrench_error_raw_pub->msg_;
    msg.header.stamp = time;
    msg.header.frame_id = Base::m_robot_base_link;
    msg.wrench = KDLWrenchToWrenchMsg(m_wrench_error_kdl_raw);
    m_wrench_error_raw_pub->unlockAndPublish();
  }  
  if (m_wrench_error_pub && m_wrench_error_pub->trylock()){
    auto& msg = m_wrench_error_pub->msg_;
    msg.header.stamp = time;
    msg.header.frame_id = Base::m_robot_base_link;
    msg.wrench = KDLWrenchToWrenchMsg(m_wrench_error_kdl);
    m_wrench_error_pub->unlockAndPublish();
  }    
}

geometry_msgs::msg::Wrench CartesianForceController::KDLWrenchToWrenchMsg(const KDL::Wrench& kdl_wrench) {
  geometry_msgs::msg::Wrench wrench_msg;
  wrench_msg.force.x  = kdl_wrench.force.x();
  wrench_msg.force.y  = kdl_wrench.force.y();
  wrench_msg.force.z  = kdl_wrench.force.z();
  wrench_msg.torque.x = kdl_wrench.torque.x();
  wrench_msg.torque.y = kdl_wrench.torque.y();
  wrench_msg.torque.z = kdl_wrench.torque.z();
  return wrench_msg;
}

// TODO: Gravity compensation placeholder for now
// void CartesianForceController::compensateGravity(const KDL::Wrench& raw_sensor_wrench)
// {
      // double m_tool_mass = 0.85; // kg (e.g., your gripper + sensor adapter)
      // KDL::Vector m_tool_com_offset(0.0, 0.0, 0.05); // m (distance from sensor to tool CoM)
      // KDL::Wrench m_gravity_compensated_wrench;

//     // 1. Get current orientation from Base FK solver (EE relative to Base)
//     KDL::Frame current_ee_pose;
//     KDL::JntArray jnts(Base::m_ik_solver->getPositions());
//     Base::m_forward_kinematics_solver->JntToCart(jnts, current_ee_pose);

//     // 2. Define Gravity vector in the Base Frame (pointing down)
//     KDL::Vector gravity_base(0.0, 0.0, -9.81);

//     // 3. Rotate the gravity vector into the End Effector frame
//     // This tells the controller which way "down" is from the robot's perspective
//     KDL::Vector gravity_ee = current_ee_pose.M.Inverse() * gravity_base;

//     // 4. Calculate the static Force and Torque exerted by the tool mass
//     KDL::Vector static_force = gravity_ee * m_tool_mass;
//     KDL::Vector static_torque = m_tool_com_offset * static_force; // Cross product (r x F)
//     KDL::Wrench tool_gravity_wrench(static_force, static_torque);

// }
}  // namespace cartesian_force_controller

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cartesian_force_controller::CartesianForceController,
                       controller_interface::ControllerInterface)
