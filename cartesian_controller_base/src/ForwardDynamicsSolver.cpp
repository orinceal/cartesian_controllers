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
  // buildGenericModel();
  m_jnt_space_inertia_solver->JntToMass(m_current_positions, m_jnt_space_inertia);

  // Compute joint jacobian
  m_jnt_jacobian_solver->JntToJac(m_current_positions, m_jnt_jacobian);
  
  // joint tip accelerations J^T f
  Eigen::VectorXd tau_tip_accel = m_jnt_jacobian.data.transpose() * net_force;

  // add additional joint forces to maintain "preferred" bias position
  // Eigen::VectorXd q_error = m_ns_positions.data - m_current_positions.data;
  // Eigen::VectorXd q_dot_error = -m_current_velocities.data; // assume q_dot_ns = 0
  // double kp_null = 1.0;
  // double kd_null = 0.2;
  // Eigen::VectorXd tau_bias_ns = kp_null * q_error + kd_null * q_dot_error;

  Eigen::VectorXd weights = Eigen::VectorXd::Ones(m_number_joints);
  weights(0) = 3.0; // higher weight for slider 
  Eigen::MatrixXd W_inv = weights.cwiseInverse().asDiagonal();
  Eigen::MatrixXd J = m_jnt_jacobian.data;
  // double lambda =  0.05;

  // J_pinv = W_inv * J^T * (J * W_inv * J^T + lambda^2 * I)^-1
  // Eigen::MatrixXd JJT_weighted = J * W_inv * J.transpose();
  // Eigen::MatrixXd damping = (lambda * lambda) * Eigen::MatrixXd::Identity(6, 6);
  // Eigen::MatrixXd J_pinv_wdls = W_inv * J.transpose() * (JJT_weighted + damping).inverse();

  // calculate null space projector
  // Eigen::MatrixXd I = Eigen::MatrixXd::Identity(m_number_joints, m_number_joints);
  // Eigen::MatrixXd P = I - (J_pinv_wdls * J);

  // Eigen::VectorXd tau_repulse = calculateRepulsionGradient();
  // // Eigen::VectorXd tau_bias_ns = calculatePosturalBias();
  // Eigen::VectorXd tau_repulse_ns = P * (tau_repulse);

  // scale damping to H^{-1} amplification
  // Eigen::VectorXd h_diag = m_jnt_space_inertia.data.diagonal();
  // double h_max = h_diag.maxCoeff();

  // calculate damping data in joint space (tau_s = -k_vq * H(q) * q_dot (see eqns 64 and 65 in https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=1087068))
  Eigen::VectorXd kvq_diag = Eigen::VectorXd::Constant(m_number_joints, m_k_vq_ns);
  // Eigen::VectorXd kvq_diag(m_number_joints);
  // for (int i = 0; i < m_number_joints; ++i) {
  //   kvq_diag(i) = m_k_vq_ns * (h_max / h_diag(i));
  //   kvq_diag(i) = std::min(kvq_diag(i), 5.0 * m_k_vq_ns);
  // }
  Eigen::MatrixXd K_vq = kvq_diag.asDiagonal();
  Eigen::VectorXd tau_s = -K_vq * m_jnt_space_inertia.data * m_last_velocities.data;
  // Khatib formulation
  // Operational space mass matrix: Lambda = (J * H^{-1} * J^T)^{-1} 
  Eigen::MatrixXd HinvJT = m_jnt_space_inertia.data.ldlt().solve(J.transpose()); // H^{-1} J^T, size: n_joints x 6
  Eigen::MatrixXd Lambda_inv = J * HinvJT; // J H^{-1} J^T, size: 6 x 6
  Eigen::MatrixXd Lambda_inv_damped = Lambda_inv + 0.001 * Eigen::MatrixXd::Identity(6, 6);

  // J^# = H^{-1} * J^T * lambda 
  Eigen::VectorXd Jsharp_f = HinvJT * Lambda_inv_damped.ldlt().solve(net_force);
  Eigen::VectorXd Hinv_tau_s = m_jnt_space_inertia.data.ldlt().solve(tau_s);

  m_current_accelerations.data = Jsharp_f + Hinv_tau_s;

  // joint accelerations according to: \f$ \ddot{q} = H^{-1} ( J^T f + B \dot{q}) \f$
  // use Cholesky decomposition (LDLT )instead of inverse to solve for q_ddot 
  // m_current_accelerations.data = m_jnt_space_inertia.data.ldlt().solve(tau_tip_accel + tau_s); // + tau_repulse_ns);
  //applyAccelLimits();
  // Numerical time integration with the Euler forward method
  m_current_velocities.data = m_last_velocities.data + m_current_accelerations.data * period.seconds();
  // m_current_velocities.data *= 0.9;  // 10 % global damping against unwanted null space motion.
  //                                    // Will cause exponential slow-down without input.

  applyVelLimits();
  
  // virtual model positions for Jacobian consistency
  m_current_positions.data = m_last_positions.data + m_current_velocities.data * period.seconds();
  // Make sure positions stay in allowed margins
  applyJointLimits();

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
  m_last_velocities = m_current_velocities;

  return control_cmd;
}

bool ForwardDynamicsSolver::init(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> nh,
                                 const KDL::Chain & chain, const KDL::JntArray & upper_pos_limits,
                                 const KDL::JntArray & lower_pos_limits,
                                 const KDL::JntArray & vel_limits,
                                 const KDL::JntArray & accel_limits)
{
  IKSolver::init(nh, chain, upper_pos_limits, lower_pos_limits, vel_limits, accel_limits);

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
  // if (total_virtual_mass > 0)f
  // {
  //   m_gravity_factor = total_real_mass / total_virtual_mass;
  // }
  // else
  // {
  //   m_gravity_factor = 1.0; 
  // }

  // RCLCPP_INFO(get_logger(), 
  //             "Gravity Factor: %.2f (Real: %.2fkg, Virtual: %.2fkg)", 
  //             m_gravity_factor, total_real_mass, total_virtual_mass);

  if (!buildGenericModel())
  {
    RCLCPP_ERROR(nh->get_logger(), "Something went wrong in setting up the internal model.");
    return false;
  }

  // Forward dynamics
  m_jnt_jacobian_solver = std::make_shared<KDL::ChainJntToJacSolver>(m_chain);
  m_jnt_space_inertia_solver = std::make_shared<KDL::ChainDynParam>(m_chain, KDL::Vector::Zero());
  // m_jnt_jacobian_solver.reset(new KDL::ChainJntToJacSolver(m_chain));
  // m_jnt_space_inertia_solver.reset(new KDL::ChainDynParam(m_chain, KDL::Vector::Zero()));

  m_jnt_jacobian.resize(m_number_joints);
  m_jnt_space_inertia.resize(m_number_joints);
  // Set the initial value if provided at runtime, else use default value.
  m_min = auto_declare(m_params + ".link_mass", 0.1);
  m_k_vq_ns = 0.0;
  // initialize null space safe joint position subscriber
  // m_ns_jnt_sub = nh->create_subscription<sensor_msgs::msg::JointState>("/optimal_joints", 1,
  // std::bind(&ForwardDynamicsSolver::nsStateCallback, this, std::placeholders::_1));

  wall_normal_.setZero();
  wall_point_.setZero();
  // initialize wall origin subscriber
  auto qos = rclcpp::QoS(1).transient_local();
  m_wall_info_sub = nh->create_subscription<geometry_msgs::msg::Pose>("/wall_info", qos,
  std::bind(&ForwardDynamicsSolver::wallPtCallback, this, std::placeholders::_1));

  RCLCPP_INFO(nh->get_logger(), "Forward dynamics solver initialized");
  RCLCPP_INFO(nh->get_logger(), "Forward dynamics solver has control over %i joints",
              m_number_joints);

  return true;
}

void ForwardDynamicsSolver::setNsDampingGain(double k_vq_ns) {
  m_k_vq_ns = k_vq_ns;
}

bool ForwardDynamicsSolver::buildGenericModel()
{
  // Set all masses and inertias to minimal (yet stable) values.
  double ip_min = 0.0001; //0.00001; //0.005; //0.002; //0.01;

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
      KDL::RigidBodyInertia(0.2, KDL::Vector::Zero(), KDL::RotationalInertia(0.2, 0.2, 0.2)));
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
  double m = 1.0;
  double ip = 1.0;
  m_chain.segments[m_chain.segments.size() - 1].setInertia(
    KDL::RigidBodyInertia(m, KDL::Vector::Zero(), KDL::RotationalInertia(ip, ip, ip)));
  return true;
}

// void ForwardDynamicsSolver::nsStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
//   if (msg->position.empty()) return;

//   // map incoming names to internal KDL order
//   for (size_t i = 0; i < msg->name.size(); ++i)
//   {
//     for (int j=0; j<m_number_joints; ++j){
//       if (msg->name[i] == m_chain.getSegment(j).getJoint().getName())
//       {
//         m_ns_positions(j) = msg->position[i];
//         break;
//       }
//     }
//   }
// }

void ForwardDynamicsSolver::wallPtCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
  wall_point_.x() = msg->position.x;
  wall_point_.y() = msg->position.y;
  wall_point_.z() = msg->position.z;
  
  Eigen::Quaterniond q(
    msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

  wall_normal_ = q * Eigen::Vector3d::UnitX();
  wall_normal_.normalize();
  RCLCPP_DEBUG(get_logger(), "wall info received. Normal: [%f, %f, %f]", wall_normal_.x(), wall_normal_.y(), wall_normal_.z());
}

Eigen::VectorXd ForwardDynamicsSolver::calculateRepulsionGradient()
{
  Eigen::VectorXd tau_repulse = Eigen::VectorXd::Zero(m_number_joints);
  
  const double rho = 0.15;  // Influence zone (15cm)
  const double d_cap = 0.02; // cap at 2cm
  const double eta = 250;  // tuning gain (stiffness)
  
  std::vector<int> joint_idx = {5, 6};
  std::vector<int> valid_seg_idx;
  valid_seg_idx.reserve(joint_idx.size());

  // get joints from elbow onwards near to wall
  for (int idx : joint_idx){
    if (idx >= 0 && idx < m_number_joints) {
      valid_seg_idx.push_back(jnt_seg_idx[idx]);
    } else {
      RCLCPP_WARN(m_handle->get_logger(), "joint index for repulsion calculation is out of bounds!");
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
    // RCLCPP_INFO(get_logger(), "distance d: %f", d);
    // calculate repulsion if within influence zone
    if (d > 0 && d < rho)
    {
      double effective_d = std::max(d, d_cap);
  
      // calculate repulsive force
      // M1: Artificial Potential Field (APF) derivative
      // double force_mag = eta * (1.0/d - 1.0/rho) * (1.0 / (d * d));
      // M2: Linear pushback
      // double force_mag = eta * (rho - d); 
      // M3: Quadratic pushback (slightly more aggressive than linear):
      double force_mag = eta * std::pow(rho - effective_d, 2);        

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

Eigen::VectorXd ForwardDynamicsSolver::calculatePosturalBias()
{
  Eigen::VectorXd tau_posture = Eigen::VectorXd::Zero(m_number_joints);
  double target_slider = 0.7;
  double target_joint_0 = -1.0310025243621384;
  // double target_joint_1 = -1.079621483313169; // 1.3197741; // joint # 2 
  // double target_joint_2 = -2.2760698537115203; // 1.6451501; // -2.4502983; // joint # 3  1.9221925081028903
  //double target_joint_3 = 0.4009423; //-1.4048959; // joint # 4 0.4048958903577902
  // double k_p = 0.05;
  double k_d = 0.005;
  tau_posture(0) = 0.1 * (target_slider - m_current_positions(0)) - k_d * m_current_velocities(0);
  tau_posture(1) = 0.07 * (target_joint_0 - m_current_positions(1)) - k_d * m_current_velocities(1);
  //tau_posture(2) = k_p * (target_joint_1 - m_current_positions(2)) - k_d * m_current_velocities(2);
  //tau_posture(3) = k_p * (target_joint_2 - m_current_positions(3)) - k_d * m_current_velocities(3);
  //tau_posture(4) = k_p * (target_joint_3 - m_current_positions(4)) - k_d * m_current_velocities(4);

  return tau_posture;
}

}  // namespace cartesian_controller_base
