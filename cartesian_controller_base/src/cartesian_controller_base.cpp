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
/*!\file    cartesian_controller_base.cpp
 *
 * \author  Jolene Ng <jolene.ng@tum.de>
 * \author  Stefan Scherzinger <scherzin@fzi.de> (Original Author)
 * \date    2026/07/15
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_controller_base/cartesian_controller_base.h>
#include <urdf/model.h>
#include <urdf_model/joint.h>

#include <cmath>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
#include "geometry_msgs/msg/detail/twist_stamped__struct.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace cartesian_controller_base
{
CartesianControllerBase::CartesianControllerBase() {}

controller_interface::InterfaceConfiguration
CartesianControllerBase::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(m_joint_names.size() * m_cmd_interface_types.size());
  for (const auto & type : m_cmd_interface_types)
  {
    for (const auto & joint_name : m_joint_names)
    {
      conf.names.push_back(joint_name + std::string("/").append(type));
    }
  }
  return conf;
}

controller_interface::InterfaceConfiguration
CartesianControllerBase::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(m_joint_names.size());  // Only position
  for (const auto & joint_name : m_joint_names)
  {
    conf.names.push_back(joint_name + "/position");
  }
  return conf;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianControllerBase::on_init()
{
  if (!m_initialized)
  {
    auto_declare<std::string>("ik_solver", "forward_dynamics");
    auto_declare<std::string>("robot_description", "");
    auto_declare<std::string>("robot_base_link", "");
    auto_declare<std::string>("end_effector_link", "");
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<std::vector<std::string>>("command_interfaces", std::vector<std::string>());
    auto_declare<bool>("solver.enable_introspection", false);
    auto_declare<double>("solver.error_scale", 1.0);
    auto_declare<int>("solver.iterations", 1);
    auto_declare<bool>("solver.publish_state_feedback", false);
    auto_declare<bool>("solver.velocity_limits_on", false);
    auto_declare<bool>("solver.vel_deadband_on", false);
    auto_declare<bool>("solver.acceleration_limits_on", false);
    auto_declare<double>("redundant_ns.trans_x.p", 0.0);
    auto_declare<double>("robot_description_planning.default_velocity_scaling_factor", 1.0);    
    auto_declare<double>("robot_description_planning.default_acceleration_scaling_factor", 1.0);
    m_initialized = true;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianControllerBase::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  if (m_configured)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // Load user specified inverse kinematics solver
  std::string ik_solver = get_node()->get_parameter("ik_solver").as_string();
  m_solver_loader.reset(new pluginlib::ClassLoader<IKSolver>(
    "cartesian_controller_base", "cartesian_controller_base::IKSolver"));
  try
  {
    m_ik_solver = m_solver_loader->createSharedInstance(ik_solver);
  }
  catch (pluginlib::PluginlibException & ex)
  {
    RCLCPP_ERROR(get_node()->get_logger(), ex.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Get kinematics specific configuration
  urdf::Model robot_model;
  KDL::Tree robot_tree;

#if defined CARTESIAN_CONTROLLERS_JAZZY
  m_robot_description = this->get_robot_description();
#else
  m_robot_description = get_node()->get_parameter("robot_description").as_string();
#endif

  if (m_robot_description.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "robot_description is empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  m_robot_base_link = get_node()->get_parameter("robot_base_link").as_string();
  if (m_robot_base_link.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "robot_base_link is empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  m_end_effector_link = get_node()->get_parameter("end_effector_link").as_string();
  if (m_end_effector_link.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "end_effector_link is empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Build a kinematic chain of the robot
  if (!robot_model.initString(m_robot_description))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse urdf model from 'robot_description'");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  if (!kdl_parser::treeFromUrdfModel(robot_model, robot_tree))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse KDL tree from urdf model");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  if (!robot_tree.getChain(m_robot_base_link, m_end_effector_link, m_robot_chain))
  {
    const std::string error =
      ""
      "Failed to parse robot chain from urdf model. "
      "Do robot_base_link and end_effector_link exist?";
    RCLCPP_ERROR(get_node()->get_logger(), error.c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Get names of actuated joints
  m_joint_names = get_node()->get_parameter("joints").as_string_array();
  if (m_joint_names.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "joints array is empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Parse joint limits
  m_number_joints = m_joint_names.size();
  KDL::JntArray upper_pos_limits(m_number_joints);
  KDL::JntArray lower_pos_limits(m_number_joints);
  KDL::JntArray vel_limits(m_number_joints);
  m_has_vel_limits = get_node()->get_parameter("solver.velocity_limits_on").as_bool();
  m_vel_scale = get_node()->get_parameter("robot_description_planning.default_velocity_scaling_factor").as_double();
  
  for (size_t i = 0; i < m_number_joints; ++i)
  {
    if (!robot_model.getJoint(m_joint_names[i]))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Joint %s does not appear in robot_description",
                   m_joint_names[i].c_str());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    if (robot_model.getJoint(m_joint_names[i])->type == urdf::Joint::CONTINUOUS)
    {
      upper_pos_limits(i) = std::nan("0");
      lower_pos_limits(i) = std::nan("0");
    }
    else
    {
      // Non-existent urdf limits are zero initialized
      upper_pos_limits(i) = robot_model.getJoint(m_joint_names[i])->limits->upper;
      lower_pos_limits(i) = robot_model.getJoint(m_joint_names[i])->limits->lower;
      std::string param_base = "robot_description_planning.joint_limits." + m_joint_names[i];
      if (m_has_vel_limits) {
        get_node()->declare_parameter<double>(param_base + ".max_velocity", 0.05);
        double limit = get_node()->get_parameter(param_base + ".max_velocity").as_double();
        vel_limits(i) = m_vel_scale * limit;
      }
    }
  }

  // Initialize solvers
  m_ik_solver->init(get_node()->shared_from_this(), m_robot_chain, upper_pos_limits, lower_pos_limits, vel_limits);
  
  KDL::Tree tmp("not_relevant");
  tmp.addChain(m_robot_chain, "not_relevant");
  m_forward_kinematics_solver.reset(new KDL::TreeFkSolverPos_recursive(tmp));
  m_iterations = get_node()->get_parameter("solver.iterations").as_int();
  m_error_scale = get_node()->get_parameter("solver.error_scale").as_double();
  double k_vq_ns = get_node()->get_parameter("redundant_ns.trans_x.p").as_double();
  // set parameters (only applies to ForwardDynamicsSolver)
  m_ik_solver->setNsDampingGain(k_vq_ns);  

  // Initialize gains k_vq for nullspace dissipation forces for redundant manipulator
  // F_rs = k_vq * x_dot according to (Khatib 1987) https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=1087068
  m_spatial_controller.init(get_node().get(), m_redundant_ns_key);

  // Check command interfaces.
  // We support position, velocity, or both.
  m_cmd_interface_types = get_node()->get_parameter("command_interfaces").as_string_array();
  if (m_cmd_interface_types.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No command_interfaces specified");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  for (const auto & type : m_cmd_interface_types)
  {
    if (type != hardware_interface::HW_IF_POSITION && type != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Unsupported command interface: %s. Choose position or velocity", type.c_str());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
  }

  // Set ROS 2 params
  m_publish_state_fb = get_node()->get_parameter("solver.publish_state_feedback").as_bool();
  m_enable_introspection = get_node()->get_parameter("solver.enable_introspection").as_bool();

  // Controller-internal state publishing
  m_feedback_pose_publisher =
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
      get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
        std::string(get_node()->get_name()) + "/current_pose", 3));

  m_feedback_twist_publisher =
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>>(
      get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(
        std::string(get_node()->get_name()) + "/current_twist", 3));

  m_joint_vel_publisher =
    std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
      get_node()->create_publisher<sensor_msgs::msg::JointState>(
        std::string(get_node()->get_name()) + "/joint_velocities", 3));
  m_filt_joint_vel_publisher =
    std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
      get_node()->create_publisher<sensor_msgs::msg::JointState>(
        std::string(get_node()->get_name()) + "/filt_joint_velocities", 3));

  // 
  m_joint_vel_publisher->msg_.name.resize(7);
  m_joint_vel_publisher->msg_.velocity.resize(7);
  m_filt_joint_vel_publisher->msg_.name.resize(7);
  m_filt_joint_vel_publisher->msg_.velocity.resize(7);
  m_joint_vel_publisher->msg_.name = {"Slider_18", "robco_joint_0", "robco_joint_1", "robco_joint_2", "robco_joint_3", "robco_joint_4", "robco_joint_5"};
  m_filt_joint_vel_publisher->msg_.name = {"Slider_18", "robco_joint_0", "robco_joint_1", "robco_joint_2", "robco_joint_3", "robco_joint_4", "robco_joint_5"};

  m_configured = true;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianControllerBase::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  stopCurrentMotion();

  if (m_active)
  {
    m_joint_cmd_pos_handles.clear();
    m_joint_cmd_vel_handles.clear();
    m_joint_state_pos_handles.clear();
    this->release_interfaces();
    m_active = false;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianControllerBase::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  if (m_active)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // Get command handles.
  for (const auto & type : m_cmd_interface_types)
  {
    if (!controller_interface::get_ordered_interfaces(command_interfaces_, m_joint_names, type,
                                                      (type == hardware_interface::HW_IF_POSITION)
                                                        ? m_joint_cmd_pos_handles
                                                        : m_joint_cmd_vel_handles))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu '%s' command interfaces, got %zu.",
                   m_joint_names.size(), type.c_str(),
                   (type == hardware_interface::HW_IF_POSITION) ? m_joint_cmd_pos_handles.size()
                                                                : m_joint_cmd_vel_handles.size());
      return CallbackReturn::ERROR;
    }
  }

  // Get state handles.
  if (!controller_interface::get_ordered_interfaces(state_interfaces_, m_joint_names,
                                                    hardware_interface::HW_IF_POSITION,
                                                    m_joint_state_pos_handles))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu '%s' state interfaces, got %zu.",
                 m_joint_names.size(), hardware_interface::HW_IF_POSITION,
                 m_joint_state_pos_handles.size());
    return CallbackReturn::ERROR;
  }

  // Copy joint state to internal simulation
  if (!m_ik_solver->setStartState(m_joint_state_pos_handles))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not set start state");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  };
  m_ik_solver->updateKinematics();

  // Provide safe command buffers with starting where we are
  computeJointControlCmds(ctrl::Vector6D::Zero(), rclcpp::Duration::from_seconds(0));
  writeJointControlCmds();

  m_active = true;
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianControllerBase::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  stopCurrentMotion();

  if (m_active)
  {
    m_joint_cmd_pos_handles.clear();
    m_joint_cmd_vel_handles.clear();
    m_joint_state_pos_handles.clear();
    this->release_interfaces();
    m_active = false;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// placeholder for now... manually set based on urdf because meshes are used
// std::vector<LinkCapsule> CartesianControllerBase::extractCollisionCapsules(
//     const urdf::Model& robot_model,
//     const KDL::Chain& chain)
// {
//     std::vector<LinkCapsule> capsules;

//     for (size_t i = 0; i < chain.segments.size(); ++i) {
//         std::string link_name = chain.segments[i].getName();
//         auto link = robot_model.getLink(link_name);

//         if (!link || !link->collision || !link->collision->geometry) continue;

//         IKSolver::LinkCapsule capsule;
//         capsule.link_name = link_name;

//         const auto& origin = link->collision->origin;
//         Eigen::Vector3d offset(
//             origin.position.x,
//             origin.position.y,
//             origin.position.z);

//         double r, p, y;
//         origin.rotation.getRPY(r, p, y);
//         Eigen::Matrix3d rot =
//             (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
//              Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
//              Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()))
//             .toRotationMatrix();

//         auto geom = link->collision->geometry;

//         if (geom->type == urdf::Geometry::CYLINDER) {
//             auto cyl = std::dynamic_pointer_cast<urdf::Cylinder>(geom);
//             Eigen::Vector3d axis = rot * Eigen::Vector3d::UnitZ();
//             double half = cyl->length / 2.0;
//             capsule.p_a    = offset + axis * half;
//             capsule.p_b    = offset - axis * half;
//             capsule.radius = cyl->radius;
//             capsule.valid  = true;

//         } else if (geom->type == urdf::Geometry::SPHERE) {
//             auto sph = std::dynamic_pointer_cast<urdf::Sphere>(geom);
//             capsule.p_a    = offset;
//             capsule.p_b    = offset;
//             capsule.radius = sph->radius;
//             capsule.valid  = true;

//         } else if (geom->type == urdf::Geometry::BOX) {
//             auto box = std::dynamic_pointer_cast<urdf::Box>(geom);
//             double dx = box->dim.x, dy = box->dim.y, dz = box->dim.z;
//             Eigen::Vector3d axis;
//             double half;
//             if (dx >= dy && dx >= dz) {
//                 axis = rot * Eigen::Vector3d::UnitX();
//                 half = dx / 2.0;
//                 capsule.radius = std::max(dy, dz) / 2.0;
//             } else if (dy >= dz) {
//                 axis = rot * Eigen::Vector3d::UnitY();
//                 half = dy / 2.0;
//                 capsule.radius = std::max(dx, dz) / 2.0;
//             } else {
//                 axis = rot * Eigen::Vector3d::UnitZ();
//                 half = dz / 2.0;
//                 capsule.radius = std::max(dx, dy) / 2.0;
//             }
//             capsule.p_a   = offset + axis * half;
//             capsule.p_b   = offset - axis * half;
//             capsule.valid = true;

//         } else if (geom->type == urdf::Geometry::MESH) {
//             // Conservative sphere fallback for mesh geometry
//             capsule.p_a    = offset;
//             capsule.p_b    = offset;
//             capsule.radius = 0.06;
//             capsule.valid  = true;
//         }

//         if (capsule.valid) {
//             capsules.push_back(capsule);
//         }
//     }

//     RCLCPP_INFO(get_node()->get_logger(),
//         "Extracted %zu collision capsules from URDF", capsules.size());

//     return capsules;
// }

void CartesianControllerBase::writeJointControlCmds()
{
  if (m_publish_state_fb)
  {
    publishStateFeedback();
  }

  auto nan_in = [](const auto & values) -> bool
  {
    for (const auto & value : values)
    {
      if (std::isnan(value))
      {
        return true;
      }
    }
    return false;
  };

  if (nan_in(m_simulated_joint_motion.positions) || nan_in(m_simulated_joint_motion.velocities))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "NaN detected in internal model. It's unlikely to recover from this. Shutting down.");
    get_node()->shutdown();
    return;
  }

  // Write all available types.
  for (const auto & type : m_cmd_interface_types)
  {
    if (type == hardware_interface::HW_IF_POSITION)
    {
      for (size_t i = 0; i < m_joint_names.size(); ++i)
      {
        (void)m_joint_cmd_pos_handles[i].get().set_value(m_simulated_joint_motion.positions[i]);
      }
    }
    if (type == hardware_interface::HW_IF_VELOCITY)
    {
      for (size_t i = 0; i < m_joint_names.size(); ++i)
      {
        (void)m_joint_cmd_vel_handles[i].get().set_value(m_simulated_joint_motion.velocities[i]);
      }
    }
  }
}
ctrl::Vector6D CartesianControllerBase::applyPDGains(const std::string & key,
                                                     const ctrl::Vector6D & error,
                                                     const ctrl::Vector6D & damping_term)
{
  // PD controlled system input with x_dot damping
  return m_spatial_controller(key, error, damping_term);
}

ctrl::Vector6D CartesianControllerBase::applyPDGains(const std::string & key,
                                                     const ctrl::Vector6D & error, 
                                                     const ctrl::Vector6D & damping_term,
                                                     const ContactState contact_state)
{
  // additional scaling in damping term based on contact state
  double kd_scale = 0.0;
  switch (contact_state) {
    case ContactState::FREE:
      kd_scale = 0.0; // no force damping when in free motion
      break;
    case ContactState::CONTACT:
      kd_scale = 1.0; // normal force damping during painting
      break;
    case ContactState::IMPACT:
      kd_scale = 3.0; // extra force damping during impact to kill bounce
      break;
  }
  return m_spatial_controller(key, error, kd_scale * damping_term);
}

void CartesianControllerBase::computeJointControlCmds(const ctrl::Vector6D & command,
                                                      const rclcpp::Duration & period)
{
  ctrl::Vector6D final_command = command;
  // Add F_rs for nullspace dissipation with P gain. No additional damping in this term!
  ctrl::Vector6D x_dot = m_ik_solver->getEndEffectorVel();
  final_command += m_spatial_controller(m_redundant_ns_key, x_dot);
  // apply error scale
  // m_error_scale = get_node()->get_parameter("solver.error_scale").as_double();
  m_cartesian_input = m_error_scale * final_command;

  // Simulate one step forward
  m_simulated_joint_motion = m_ik_solver->getJointControlCmds(period, m_cartesian_input);

  m_ik_solver->updateKinematics(); // change to update before computing errors in loop
}

ctrl::Vector6D CartesianControllerBase::displayInBaseLink(const ctrl::Vector6D & vector,
                                                          const std::string & from)
{
  // Adjust format
  KDL::Wrench wrench_kdl;
  for (int i = 0; i < 6; ++i)
  {
    wrench_kdl(i) = vector[i];
  }
  
  KDL::Frame transform_kdl;
  m_forward_kinematics_solver->JntToCart(m_ik_solver->getPositions(), transform_kdl, from);

  // Rotate into new reference frame
  wrench_kdl = transform_kdl.M * wrench_kdl;

  // Reassign
  ctrl::Vector6D out;
  for (int i = 0; i < 6; ++i)
  {
    out[i] = wrench_kdl(i);
  }

  return out;
}

ctrl::Matrix6D CartesianControllerBase::displayInBaseLink(const ctrl::Matrix6D & tensor,
                                                          const std::string & from)
{
  // Get rotation to base
  KDL::Frame R_kdl;
  m_forward_kinematics_solver->JntToCart(m_ik_solver->getPositions(), R_kdl, from);

  // Adjust format
  ctrl::Matrix3D R;
  R << R_kdl.M.data[0], R_kdl.M.data[1], R_kdl.M.data[2], R_kdl.M.data[3], R_kdl.M.data[4],
    R_kdl.M.data[5], R_kdl.M.data[6], R_kdl.M.data[7], R_kdl.M.data[8];

  // Treat diagonal blocks as individual 2nd rank tensors.
  // Display in base frame.
  ctrl::Matrix6D tmp = ctrl::Matrix6D::Zero();
  tmp.topLeftCorner<3, 3>() = R * tensor.topLeftCorner<3, 3>() * R.transpose();
  tmp.bottomRightCorner<3, 3>() = R * tensor.bottomRightCorner<3, 3>() * R.transpose();

  return tmp;
}

KDL::Wrench CartesianControllerBase::displayInBaseLink(const KDL::Wrench & wrench,
                                                          const std::string & from)
{
  KDL::Frame transform_kdl;
  m_forward_kinematics_solver->JntToCart(m_ik_solver->getPositions(), transform_kdl, from);

  // Rotate into new reference frame
  return transform_kdl.M * wrench;
}

ctrl::Vector6D CartesianControllerBase::displayInTipLink(const ctrl::Vector6D & vector,
                                                         const std::string & to)
{
  // Adjust format
  KDL::Wrench wrench_kdl;
  for (int i = 0; i < 6; ++i)
  {
    wrench_kdl(i) = vector[i];
  }

  KDL::Frame transform_kdl;
  m_forward_kinematics_solver->JntToCart(m_ik_solver->getPositions(), transform_kdl, to);

  // Rotate into new reference frame
  wrench_kdl = transform_kdl.M.Inverse() * wrench_kdl;

  // Reassign
  ctrl::Vector6D out;
  for (int i = 0; i < 6; ++i)
  {
    out[i] = wrench_kdl(i);
  }

  return out;
}

KDL::Rotation CartesianControllerBase::rotationToBase(const std::string & from) 
{
  KDL::Frame transform_kdl;
  m_forward_kinematics_solver->JntToCart(m_ik_solver->getPositions(), transform_kdl, from);
  return transform_kdl.M;
}

void CartesianControllerBase::updateIntrospectionVector(const KDL::Frame & frame, ctrl::Vector6D & target_vector)
{
  target_vector(0) = frame.p.x();
  target_vector(1) = frame.p.y();
  target_vector(2) = frame.p.z();

  double r, p, y;
  frame.M.GetRPY(r, p, y);
  target_vector(3) = r;
  target_vector(4) = p;
  target_vector(5) = y;
}

void CartesianControllerBase::publishStateFeedback()
{
  // End-effector pose
  auto pose = m_ik_solver->getEndEffectorPose();
  if (m_feedback_pose_publisher->trylock())
  {
    m_feedback_pose_publisher->msg_.header.stamp = get_node()->now();
    m_feedback_pose_publisher->msg_.header.frame_id = m_robot_base_link;
    m_feedback_pose_publisher->msg_.pose.position.x = pose.p.x();
    m_feedback_pose_publisher->msg_.pose.position.y = pose.p.y();
    m_feedback_pose_publisher->msg_.pose.position.z = pose.p.z();

    pose.M.GetQuaternion(m_feedback_pose_publisher->msg_.pose.orientation.x,
                         m_feedback_pose_publisher->msg_.pose.orientation.y,
                         m_feedback_pose_publisher->msg_.pose.orientation.z,
                         m_feedback_pose_publisher->msg_.pose.orientation.w);

    m_feedback_pose_publisher->unlockAndPublish();
  }

  // End-effector twist
  auto twist = m_ik_solver->getEndEffectorVel();
  if (m_feedback_twist_publisher->trylock())
  {
    m_feedback_twist_publisher->msg_.header.stamp = get_node()->now();
    m_feedback_twist_publisher->msg_.header.frame_id = m_robot_base_link;
    m_feedback_twist_publisher->msg_.twist.linear.x = twist[0];
    m_feedback_twist_publisher->msg_.twist.linear.y = twist[1];
    m_feedback_twist_publisher->msg_.twist.linear.z = twist[2];
    m_feedback_twist_publisher->msg_.twist.angular.x = twist[3];
    m_feedback_twist_publisher->msg_.twist.angular.y = twist[4];
    m_feedback_twist_publisher->msg_.twist.angular.z = twist[5];

    m_feedback_twist_publisher->unlockAndPublish();
  }
  
  auto joint_vel = m_ik_solver->getJointVel();
  auto filt_joint_vel = m_ik_solver->getFiltJointVel();
  if (m_joint_vel_publisher->trylock()) {
    m_joint_vel_publisher->msg_.header.stamp = get_node()->now();
    for (int i = 0; i < 7; ++i) {
      m_joint_vel_publisher->msg_.velocity[i] = joint_vel(i);
    }
    m_joint_vel_publisher->unlockAndPublish();
  }
  if (m_filt_joint_vel_publisher->trylock()) {
    m_filt_joint_vel_publisher->msg_.header.stamp = get_node()->now();
    for (int i = 0; i < 7; ++i) {
      m_filt_joint_vel_publisher->msg_.velocity[i] = filt_joint_vel(i);
    }
    m_filt_joint_vel_publisher->unlockAndPublish();
  }
}

}  // namespace cartesian_controller_base
