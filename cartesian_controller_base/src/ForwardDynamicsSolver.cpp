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

Eigen::Vector3d ForwardDynamicsSolver::getWallNormal() const { return wall_normal_; }

trajectory_msgs::msg::JointTrajectoryPoint ForwardDynamicsSolver::getJointControlCmds(
  const rclcpp::Duration & period, const ctrl::Vector6D & net_force)
{  
  // Compute joint space inertia matrix
  m_jnt_space_inertia_solver->JntToMass(m_current_positions, m_jnt_space_inertia);

  // Compute joint jacobian
  m_jnt_jacobian_solver->JntToJac(m_current_positions, m_jnt_jacobian);
  Eigen::MatrixXd J = m_jnt_jacobian.data;  
  
  // joint tip accelerations J^T f
  Eigen::VectorXd tau_tip_accel = J.transpose() * net_force;

  // calculate collision repulsion and get min clearance for all capsules
  Eigen::VectorXd tau_repulse = Eigen::VectorXd::Zero(m_number_joints);
  addCollisionRepulsion(tau_repulse, period); 

  // calculate postural bias for slider
  // const auto tau_posture = calculatePosturalBias();

  // Khatib formulation
  // Operational space mass matrix: Lambda = (J * H^{-1} * J^T)^{-1} 
  Eigen::MatrixXd HinvJT = m_jnt_space_inertia.data.ldlt().solve(J.transpose()); // H^{-1} J^T, size: n_joints x 6
  Eigen::MatrixXd Lambda_inv = J * HinvJT + 1e-6 * Eigen::MatrixXd::Identity(6, 6); // J H^{-1} J^T damped, size: 6 x 6

  // calculate null space damping (tau_d = -k_vq * H(q) * q_dot (see eqns 64 and 65 in https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=1087068))
  Eigen::VectorXd tau_d = -m_k_vq_ns * m_jnt_space_inertia.data * m_last_velocities.data;

  // torques that don't affect tip 
  Eigen::VectorXd tau_sec = tau_repulse + tau_d; // + tau_posture;

  // dynamically consistent projection N^T tau = tau - J^T * lambda * (J H ^-1 tau)
  // J^# = H^{-1} * J^T * lambda 
  Eigen::VectorXd Hinv_tau_s = m_jnt_space_inertia.data.ldlt().solve(tau_sec);    // H^-1 tau_null (n)
  Eigen::VectorXd tip_couple = J * Hinv_tau_s;                                     // leaked tip accel
  Eigen::VectorXd tau_null_ns = tau_sec - J.transpose() * Lambda_inv.ldlt().solve(tip_couple);

  // joint accelerations according to: \f$ \ddot{q} = H^{-1} ( J^T f + B \dot{q}) \f$
  // use Cholesky decomposition (LDLT )instead of inverse to solve for q_ddot 
  m_current_accelerations.data = m_jnt_space_inertia.data.ldlt().solve(tau_tip_accel + tau_null_ns); // + tau_s); //+ tau_repulse_ns);

  // Numerical time integration with the Euler forward method
  m_current_velocities.data = m_last_velocities.data + m_current_accelerations.data * period.seconds();
  m_last_velocities = m_current_velocities; // save pre-deadband for next integration

  // Apply q_dot limits and additional damping if necessary                                     
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

  return control_cmd;
}

bool ForwardDynamicsSolver::init(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> nh,
                                 const KDL::Chain & chain, const KDL::JntArray & upper_pos_limits,
                                 const KDL::JntArray & lower_pos_limits,
                                 const KDL::JntArray & vel_limits)
{
  IKSolver::init(nh, chain, upper_pos_limits, lower_pos_limits, vel_limits);

  if (!buildGenericModel())
  {
    RCLCPP_ERROR(nh->get_logger(), "Something went wrong in setting up the internal model.");
    return false;
  }

  // Forward dynamics
  m_jnt_jacobian_solver = std::make_shared<KDL::ChainJntToJacSolver>(m_chain);
  m_jnt_space_inertia_solver = std::make_shared<KDL::ChainDynParam>(m_chain, KDL::Vector::Zero());

  m_jnt_jacobian.resize(m_number_joints);
  m_jnt_space_inertia.resize(m_number_joints);
  m_k_vq_ns = 0.0;

  // Define collision capsules. Dimensions measured from STL meshes
  // find segment index by link name
  auto findSegIdx = [this, nh](const std::string& link_name) -> int {
      for (size_t i = 0; i < m_chain.segments.size(); ++i) {
          if (m_chain.segments[i].getName() == link_name) {
              return static_cast<int>(i);
          }
      }
      RCLCPP_WARN(nh->get_logger(),
          "Link %s not found in chain", link_name.c_str());
      return -1;
  };

  // D116_customer_v1_1 — parent of joint_4 (motor body)
  // CoM at (0.0635, 0, -0.013) — mostly along x
  LinkCapsule joint_4_motor;
  joint_4_motor.link_name = "D116_customer_v1_1";
  joint_4_motor.p_a       = Eigen::Vector3d(0.0603,  0.0000,  0.0772);
  joint_4_motor.p_b       = Eigen::Vector3d(0.0603,  0.0000, -0.0968);
  joint_4_motor.radius    = 0.0698;
  joint_4_motor.seg_idx   = findSegIdx(joint_4_motor.link_name);
  joint_4_motor.valid     = joint_4_motor.seg_idx >= 0;
  if (joint_4_motor.valid) collision_capsules_.push_back(joint_4_motor);

  // D116_customer_v1_2 — parent of joint_5, lanze — child of joint_5
  // LinkCapsule joint_5_motor;
  // joint_5_motor.link_name = "D116_customer_v1_2";
  // joint_5_motor.p_a       = Eigen::Vector3d(0.0001,  0.0619, 0.0631);
  // joint_5_motor.p_b       = Eigen::Vector3d(0.0001, -0.0978, 0.0631);
  // joint_5_motor.radius    = 0.0699;
  // joint_5_motor.seg_idx = findSegIdx(joint_5_motor.link_name);
  // joint_5_motor.valid     = joint_5_motor.seg_idx >= 0;
  // if (joint_5_motor.valid) collision_capsules_.push_back(joint_5_motor);

  RCLCPP_INFO(nh->get_logger(),
      "Initialized %zu collision capsules", collision_capsules_.size());

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
      //KDL::RigidBodyInertia(0.1, KDL::Vector::Zero(), KDL::RotationalInertia(0.1, 0.1, 0.1)));
      KDL::RigidBodyInertia(0.0001, KDL::Vector::Zero(), KDL::RotationalInertia(0.0001, 0.0001, 0.0001)));
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
  
  Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

  wall_normal_ = q * Eigen::Vector3d::UnitX(); // normal points out of wall. Defined as UnitX() col0 of rotation matrix in WallTrajectory
  wall_normal_.normalize();

  RCLCPP_INFO(get_logger(), "wall info received. Point on wall: [%f, %f, %f], Normal: [%f, %f, %f]", wall_point_.x(), wall_point_.y(), wall_point_.z(), wall_normal_.x(), wall_normal_.y(), wall_normal_.z());

  initializeCapsuleClearances();

  // detect normal angle change (for corners)
  // // Detect corner: angle between new and existing normal exceeds threshold
  // double cos_angle = wall_normal_.dot(new_normal);
  // bool is_corner = (cos_angle < std::cos(corner_angle_threshold_rad_)); // e.g. 30°

  // if (is_corner && wall_normal_.norm() > 0.1) {
  //   // Store second wall normal
  //   secondary_wall_normal_ = new_normal;
  //   in_corner_ = true;
  //   RCLCPP_INFO(get_logger(), "Corner detected! Angle: %.1f deg",
  //               std::acos(cos_angle) * 180.0 / M_PI);
  // } else {
  //   // Normal flat wall update
  //   wall_normal_ = new_normal;
  //   wall_point_.x() = msg->position.x;
  //   wall_point_.y() = msg->position.y;
  //   wall_point_.z() = msg->position.z;
  //   in_corner_ = false;
  // }
}

void ForwardDynamicsSolver::initializeCapsuleClearances()
{
    if (wall_normal_.isZero()) {
      // no wall normal set for wall collision checking
        return;
    }
    for (auto& capsule : collision_capsules_) {
        if (!capsule.valid || capsule.seg_idx < 0) continue;
        
        double clearance_a, clearance_b;
        computeCapsuleClearance(capsule, clearance_a, clearance_b);
        
        capsule.last_d_a = clearance_a;
        capsule.last_d_b = clearance_b;
        capsule.initialized = true;
        RCLCPP_INFO(m_handle->get_logger(),
            "Initialized %s: clearance_a=%.3fm clearance_b=%.3fm",
            capsule.link_name.c_str(), clearance_a, clearance_b);
    }
}
void ForwardDynamicsSolver::computeCapsuleClearance(const LinkCapsule& capsule, double & clearance_a, double & clearance_b)
{
  // Get segment frame
  KDL::Frame joint_frame;
  m_fk_pos_solver->JntToCart(m_current_positions, joint_frame, capsule.seg_idx);
  
  Eigen::Matrix3d R;
  for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
          R(i, j) = joint_frame.M.data[i * 3 + j];
  Eigen::Vector3d t(joint_frame.p.x(), joint_frame.p.y(), joint_frame.p.z());

  // transform endpoints to base frame
  Eigen::Vector3d p_a_base = R * capsule.p_a + t;
  Eigen::Vector3d p_b_base = R * capsule.p_b + t;

  // signed distance from capsule endpoints to wall plane
  double d_a = (p_a_base - wall_point_).dot(wall_normal_);
  double d_b = (p_b_base - wall_point_).dot(wall_normal_);

  clearance_a = d_a - capsule.radius;
  clearance_b = d_b - capsule.radius;
}

void ForwardDynamicsSolver::addCollisionRepulsion(Eigen::VectorXd& tau_repulse, const rclcpp::Duration& period)
{
  // double min_clearance = std::numeric_limits<double>::max();
  double clearance_dot, endpt_clearance;
  if (collision_capsules_.empty()) return;
  if (wall_normal_.isZero()) return;

  const double clearance_zone = 0.1;     // influence zone
  const double rho = 0.06;
  const double eta = 25.0;   // stiffness coeff
  const double beta = 0.6; // damping coeff
  const double d_cap = 0.03; // hard limit
  // const double dot_eps = 0.001; // ignore approach speeds below this
  double dt = period.seconds();

  for (auto& capsule : collision_capsules_) {
      if (!capsule.valid || capsule.seg_idx < 0) continue;

      // get min clearance and clearance_dot for capsule endpoints
      double clearance_a, clearance_b;
      computeCapsuleClearance(capsule, clearance_a, clearance_b);
      double clearance_dot_a = 0.0;
      double clearance_dot_b = 0.0;
      
      if (capsule.initialized) {
      clearance_dot_a = (clearance_a - capsule.last_d_a) / dt;
      clearance_dot_b = (clearance_b - capsule.last_d_b) / dt;

      // filter derivaives
      capsule.filt_dot_a = 0.7 * capsule.filt_dot_a + 0.3 * clearance_dot_a;
      capsule.filt_dot_b = 0.7 * capsule.filt_dot_b + 0.3 * clearance_dot_b;
      } 

      // pick endpoint rotating in (most negative rate)
      if (clearance_dot_a < clearance_dot_b) {  
          endpt_clearance = clearance_a;
          clearance_dot       = capsule.filt_dot_a;
      } else {
          endpt_clearance = clearance_b;
          clearance_dot       = capsule.filt_dot_b;
      }

      // push endpoint if within zone and approaching
      if (endpt_clearance < clearance_zone && clearance_dot < 0.0) {
          double effective = std::max(endpt_clearance, d_cap);
          double spring_force = std::max(eta * (rho - effective), 0.0);  // linear    
          // double spring_force = eta * std::pow(rho - effective, 2);                                // quadratic 
          double fade = std::clamp((clearance_zone - endpt_clearance) / (clearance_zone - rho), 0.0, 1.0);
          double damping_force = fade * (-beta * clearance_dot);
          double force_mag = std::max(spring_force + damping_force, 0.0);
          Eigen::Vector3d f_repulse = force_mag * wall_normal_; // positive force away from wall

          // Jacobian at this segment
          KDL::Jacobian J_seg(m_number_joints);
          m_jnt_jacobian_solver->JntToJac(
              m_current_positions, J_seg, capsule.seg_idx);
          tau_repulse += J_seg.data.topRows(3).transpose() * f_repulse; // only linear forces -> joint torque
          // RCLCPP_INFO_THROTTLE(get_logger(), *m_handle->get_clock(), 500,
          // "clearance_d_min: %.4f, clearance_dot: %.4f, spring_force: %.4f, damping_force: %.4f ", clearance_d_min, clearance_dot, spring_force, damping_force);
      } 

      capsule.last_d_a = clearance_a;
      capsule.last_d_b = clearance_b;
  }
  return;
}

// OPTIONAL current implementation for postural bias. need to subscribe to target pose... 
Eigen::VectorXd ForwardDynamicsSolver::calculatePosturalBias()
{
  Eigen::VectorXd tau_posture = Eigen::VectorXd::Zero(m_number_joints);
  // Eigen::VectorXd q_ref = Eigen::VectorXd::Zero(m_number_joints);
  // q_ref(0) = 0.4;
  // q_ref(1) = 2.267;
  // q_ref(2) = 0.892;
  // q_ref(3) = 2.318;
  // q_ref(4) = -0.127;
  // q_ref(5) = -1.605;
  // q_ref(6) = 1.961;
  double target_slider = 0.5;
  // double target_joint_0 = -1.0310025243621384;
  double k_p = 0.1;
  double k_d = 0.5;
  //double posture_deadband = 0.2;
  double joint_deadband = 0.2;
  double err_slider = target_slider - m_current_positions(0);
  if (std::abs(err_slider) > joint_deadband) {
    tau_posture(0) = k_p * err_slider - k_d * m_current_velocities(0);
  }
  
  // for (int i = 1; i < m_number_joints; ++i) {
  //   double err = q_ref(i) - m_current_positions(i);
  //   if (std::abs(err) > joint_deadband) {
  //   tau_posture(i) = k_p * err - k_d * m_current_velocities(i);
  //   }
  // }
  //tau_posture(1) = 0.07 * (target_joint_0 - m_current_positions(1)) - k_d * m_current_velocities(1);
  //tau_posture(2) = k_p * (target_joint_1 - m_current_positions(2)) - k_d * m_current_velocities(2);
  //tau_posture(3) = k_p * (target_joint_2 - m_current_positions(3)) - k_d * m_current_velocities(3);
  //tau_posture(4) = k_p * (target_joint_3 - m_current_positions(4)) - k_d * m_current_velocities(4);
  // tau_posture(6) = k_p * (target_joint_5 - m_current_positions(6)) - k_d * m_current_velocities(6);
  return tau_posture;
}

}  // namespace cartesian_controller_base
