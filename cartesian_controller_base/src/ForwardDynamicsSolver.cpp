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
/*!\file    ForwardDynamicsSolver.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2020/03/24
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_controller_base/ForwardDynamicsSolver.h>

#include <algorithm>
#include <kdl/framevel.hpp>
#include <kdl/jntarrayvel.hpp>
#include <map>
#include <pluginlib/class_list_macros.hpp>
#include <sstream>

/**
 * \class cartesian_controller_base::ForwardDynamicsSolver
 *
 * Users may explicitly specify it with \a "forward_dynamics" as \a ik_solver
 * in their controllers.yaml configuration file for each controller:
 *
 * \code{.yaml}
 * <name_of_your_controller>:
 *   ros__parameters:
 *     ik_solver: "forward_dynamics"
 *     ...
 *
 *     solver:
 *         ...
 *         forward_dynamics:
 *             link_mass: 0.5
 * \endcode
 *
 */
PLUGINLIB_EXPORT_CLASS(cartesian_controller_base::ForwardDynamicsSolver,
                       cartesian_controller_base::IKSolver)

namespace cartesian_controller_base
{
ForwardDynamicsSolver::ForwardDynamicsSolver() {}

ForwardDynamicsSolver::~ForwardDynamicsSolver() {}

trajectory_msgs::msg::JointTrajectoryPoint ForwardDynamicsSolver::getJointControlCmds(
  rclcpp::Duration period, const ctrl::Vector6D & net_force)
{  
  // Compute joint space inertia matrix with actualized link masses
  buildGenericModel();
  m_jnt_space_inertia_solver->JntToMass(m_current_positions, m_jnt_space_inertia);

  // Compute joint jacobian
  m_jnt_jacobian_solver->JntToJac(m_current_positions, m_jnt_jacobian);

  // Eigen::VectorXd tau_gravity = Eigen::VectorXd::Zero(m_number_joints);
  // double gravity = 9.81;
  // Eigen::VectorXd joint_scales = Eigen::VectorXd::Constant(m_number_joints, m_gravity_factor);
  // //joint_scales(0) = 1.0;

  // for (size_t i = 0; i < m_chain.segments.size(); ++i)
  // {
  //   double virtual_mass = m_chain.segments[i].getInertia().getMass();
  //   if (virtual_mass <= 1e-6) continue;

  //   KDL::Jacobian J_link(m_number_joints);
  //   m_jnt_jacobian_solver->JntToJac(m_current_positions, J_link, i+1);
  
  //   ctrl::Vector6D wrench_support;
  //   wrench_support << 0.0, 0.0, (virtual_mass * gravity), 0.0, 0.0, 0.0;

  //   // accumulate torque, tau_gravity = J^T * Wrench
  //   tau_gravity += J_link.data.transpose() * wrench_support;
  // }
  // Eigen::VectorXd tau_gravity_scaled = tau_gravity.cwiseProduct(joint_scales);
  // evaluate null space forces
  // if (m_ns_positions.rows() != m_current_positions.rows()) {
  //   return 
  // }
  // add additional damping from redundant manipulator F_ext
  // Eigen::VectorXd q_error = m_ns_positions.data - m_current_positions.data;
  // Eigen::VectorXd q_dot_error = -m_current_velocities.data; // assume q_dot_ns = 0
  // double kp_null = 1.0;
  // double kd_null = 0.2;
  // Eigen::VectorXd null_space_bias = kp_null * q_error + kd_null * q_dot_error;

  Eigen::VectorXd weights = Eigen::VectorXd::Ones(m_number_joints);
  //weights(0) = 20.0; // higher weight for slider 
  Eigen::MatrixXd W_inv = weights.cwiseInverse().asDiagonal();
  Eigen::MatrixXd J = m_jnt_jacobian.data;
  double lambda = 0.05;

  // J_pinv = W_inv * J^T * (J * W_inv * J^T + lambda^2 * I)^-1
  Eigen::MatrixXd JJT_weighted = J * W_inv * J.transpose();
  Eigen::MatrixXd damping = (lambda * lambda) * Eigen::MatrixXd::Identity(6, 6);
  Eigen::MatrixXd J_pinv_wdls = W_inv * J.transpose() * (JJT_weighted + damping).inverse();

  // calculate null space projector
  //Eigen::MatrixXd J_pinv = m_jnt_jacobian.data.completeOrthogonalDecomposition().pseudoInverse();
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(m_number_joints, m_number_joints);
  //Eigen::MatrixXd P = I - (J_pinv * m_jnt_jacobian.data);
  Eigen::MatrixXd P = I - (J_pinv_wdls * J);
  // Eigen::VectorXd tau_ns = P * null_space_bias;
  Eigen::VectorXd tau_repulse = calculateRepulsionGradient();
  Eigen::VectorXd tau_nullsp = P * tau_repulse;

  // joint tip accelerations J^T f
  Eigen::VectorXd tau_tip_accel = m_jnt_jacobian.data.transpose() * net_force;

  // calculate damping data
  Eigen::VectorXd damping_coeffs = Eigen::VectorXd::Constant(m_number_joints, 0.002);
  damping_coeffs(0) = 4.0;
  Eigen::VectorXd tau_damping = damping_coeffs.cwiseProduct(m_last_velocities.data);

  // joint accelerations according to: \f$ \ddot{q} = H^{-1} ( J^T f + B \dot{q}) \f$
  m_current_accelerations.data = m_jnt_space_inertia.data.inverse() * (tau_tip_accel + tau_damping); //+ tau_ns);
  //applyAccelLimits();
  // Numerical time integration with the Euler forward method
  m_current_velocities.data = m_last_velocities.data + m_current_accelerations.data * period.seconds();
  m_current_velocities.data *= 0.9;  // 10 % global damping against unwanted null space motion.
                                     // Will cause exponential slow-down without input.
  applyVelLimits();
  m_current_positions.data = m_last_positions.data + m_current_velocities.data * period.seconds();
  // RCLCPP_INFO(nh_->get_logger(), "positions: %f %f %f %f %f %f %f", m_current_positions(0), m_current_positions(1), m_current_positions(2),
  //             m_current_positions(3), m_current_positions(4), m_current_positions(5), m_current_positions(6));

  // Make sure positions stay in allowed margins
  applyJointLimits();
  // RCLCPP_INFO(nh_->get_logger(), "velocities: %f %f %f %f %f %f %f", m_current_velocities.data(0), m_current_velocities.data(1), m_current_velocities.data(2),
  //             m_current_velocities.data(3), m_current_velocities.data(4), m_current_velocities.data(5), m_current_velocities.data(6));
  // Apply results
  trajectory_msgs::msg::JointTrajectoryPoint control_cmd;
  for (int i = 0; i < m_number_joints; ++i)
  {
    control_cmd.positions.push_back(m_current_positions(i));
    control_cmd.velocities.push_back(m_current_velocities(i));

    // Accelerations should be left empty. Those values will be interpreted
    // by most hardware joint drivers as max. tolerated values. As a
    // consequence, the robot will move very slowly.
  }
  control_cmd.time_from_start = period;  // valid for this duration

  // Update for the next cycle
  m_last_positions = m_current_positions;
  m_last_velocities = m_current_velocities; // assume last velocity as zero, instantaneous acceleration

  return control_cmd;
}

bool ForwardDynamicsSolver::init(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> nh,
                                 const KDL::Chain & chain, const KDL::JntArray & upper_pos_limits,
                                 const KDL::JntArray & lower_pos_limits,
                                 const KDL::JntArray & vel_limits,
                                 const KDL::JntArray & accel_limits)
{
  IKSolver::init(nh, chain, upper_pos_limits, lower_pos_limits, vel_limits, accel_limits);
  nh_ = nh;

  // double total_real_mass = 0.0;
  // double total_virtual_mass = 0.0;

  // for (size_t i = 0; i < m_chain.getNrOfSegments(); ++i)
  // {
  //   // 1. Accumulate Real Mass from URDF
  //   double m_real = m_chain.getSegment(i).getInertia().getMass();
  //   total_real_mass += m_real;

  //   // 2. Predict Virtual Mass (mirroring buildGenericModel logic)
  //   if (i == m_chain.getNrOfSegments() - 1)
  //   {
  //     total_virtual_mass += 1.0;
  //   } else if (m_chain.getSegment(i).getJoint().getType() != KDL::Joint::None) {
  //     total_virtual_mass += 0.1;
  //   }
  // }
  // // 3. Calculate the Balloon Factor
  // // Avoid division by zero
  // if (total_virtual_mass > 0)
  // {
  //   m_gravity_factor = total_real_mass / total_virtual_mass;
  // }
  // else
  // {
  //   m_gravity_factor = 1.0; 
  // }

  // RCLCPP_INFO(nh_->get_logger(), 
  //             "Gravity Factor: %.2f (Real: %.2fkg, Virtual: %.2fkg)", 
  //             m_gravity_factor, total_real_mass, total_virtual_mass);

  if (!buildGenericModel())
  {
    RCLCPP_ERROR(nh->get_logger(), "Something went wrong in setting up the internal model.");
    return false;
  }

  // Forward dynamics
  m_jnt_jacobian_solver.reset(new KDL::ChainJntToJacSolver(m_chain));
  m_jnt_space_inertia_solver.reset(new KDL::ChainDynParam(m_chain, KDL::Vector::Zero()));
  // m_jnt_space_gravity_solver.reset(new KDL::ChainDynParam(m_chain, KDL::Vector::Zero()));
  m_jnt_jacobian.resize(m_number_joints);
  m_jnt_space_inertia.resize(m_number_joints);

  // Set the initial value if provided at runtime, else use default value.
  m_min = auto_declare(m_params + ".link_mass", 0.1);

  // initialize null space safe joint position subscriber
  m_ns_jnt_sub = nh->create_subscription<sensor_msgs::msg::JointState>("/optimal_joints", 1,
  std::bind(&ForwardDynamicsSolver::nsStateCallback, this, std::placeholders::_1));
  
  RCLCPP_INFO(nh->get_logger(), "Forward dynamics solver initialized");
  RCLCPP_INFO(nh->get_logger(), "Forward dynamics solver has control over %i joints",
              m_number_joints);

  return true;
}

bool ForwardDynamicsSolver::buildGenericModel()
{
  // Set all masses and inertias to minimal (yet stable) values.
  double ip_min = 0.001; //0.00001; //0.005; //0.002; //0.01;

  jnt_seg_idx.resize(m_number_joints);
  int j = 0;
  for (size_t i = 0; i < m_chain.segments.size(); ++i)  
  {
    // Fixed joint segment
    if (m_chain.segments[i].getJoint().getType() == KDL::Joint::None)
    {
      m_chain.segments[i].setInertia(KDL::RigidBodyInertia::Zero());
    }
    else if (m_chain.segments[i].getJoint().getType() == KDL::Joint::TransAxis) {
      // set higher mass for slider joint
      m_chain.segments[i].setInertia(
      KDL::RigidBodyInertia(0.7, KDL::Vector::Zero(), KDL::RotationalInertia(0.7, 0.7, 0.7)));
      jnt_seg_idx.at(j) = i;
      j++;
    } 
    else  // relatively moving segment
    {
      m_chain.segments[i].setInertia(
        KDL::RigidBodyInertia(m_min,                          // mass
                              KDL::Vector::Zero(),            // center of gravity
                              KDL::RotationalInertia(ip_min,  // ixx
                                                     ip_min,  // iyy
                                                     ip_min   // izz
                                                     // ixy, ixy, iyz default to 0.0
                                                     )));
      jnt_seg_idx.at(j) = i;
      j++;
    }
  }

  // Only give the last segment a generic mass and inertia.
  // See https://arxiv.org/pdf/1908.06252.pdf for a motivation for this setting.
  double m = 1;
  double ip = 1;
  m_chain.segments[m_chain.segments.size() - 1].setInertia(
    KDL::RigidBodyInertia(m, KDL::Vector::Zero(), KDL::RotationalInertia(ip, ip, ip)));
  return true;
}

void ForwardDynamicsSolver::nsStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (msg->position.empty()) return;

  // map incoming names to internal KDL order
  for (size_t i = 0; i < msg->name.size(); ++i)
  {
    for (int j=0; j<m_number_joints; ++j){
      if (msg->name[i] == m_chain.getSegment(j).getJoint().getName())
      {
        m_ns_positions(j) = msg->position[i];
        break;
      }
    }
  }
}
Eigen::VectorXd ForwardDynamicsSolver::calculateRepulsionGradient()
{
  Eigen::VectorXd tau_repulse = Eigen::VectorXd::Zero(m_number_joints);
  
  const double rho = 0.15;  // Influence zone (15cm)
  const double eta = 0.01;  // Scaling gain
  
  std::vector<int> joint_idx = {5, 6};
  std::vector<int> valid_seg_idx;
  valid_seg_idx.reserve(joint_idx.size());

  // get joints from elbow onwards near to wall
  for (int idx : joint_idx){
    if (idx >= 0 && idx < m_number_joints) {
      valid_seg_idx.push_back(jnt_seg_idx[idx]);
    } else {
      RCLCPP_WARN(nh_->get_logger(), "joint index for repulsion calculation is out of bounds!");
      return tau_repulse;
    }
  }
  for (int seg_idx : valid_seg_idx) {
    KDL::Frame segment_frame;

    m_fk_pos_solver->JntToCart(m_current_positions, segment_frame, seg_idx);
    Eigen::Vector3d p_segment(segment_frame.p.x(), segment_frame.p.y(), segment_frame.p.z());

    // calculate distance to wall by projecting on normal 
    // d = (P_link - P_wall) . n
    double d = (p_segment - wall_point_).dot(wall_normal_);

    // calculate repulsion if within influence zone
    if (d > 0 && d < rho)
    {
      // calculate repulsive force
      double force_mag = eta * (1.0/d - 1.0/rho) * (1.0 / (d * d));
        
      // force vector points along normal
      Eigen::Vector3d f_cartesian = force_mag * wall_normal_;

      // map to Joint Space
      KDL::Jacobian J_segment(m_number_joints); 
      m_jnt_jacobian_solver->JntToJac(m_current_positions, J_segment, seg_idx);
      // project the 3D force through top 3 rows (linear part) of the Jacobian
      tau_repulse += J_segment.data.topRows(3).transpose() * f_cartesian;
    }
  }
  return tau_repulse;
}

}  // namespace cartesian_controller_base
