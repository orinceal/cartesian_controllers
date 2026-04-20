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
/*!\file    SpatialPDController.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/28
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_controller_base/SpatialPDController.h>

#include <string>

namespace cartesian_controller_base
{
SpatialPDController::SpatialPDController() {}

ctrl::Vector6D SpatialPDController::operator()(const std::string & key, const ctrl::Vector6D & error) {
  return (*this)(key, error, ctrl::Vector6D::Zero());
}

ctrl::Vector6D SpatialPDController::operator()(const std::string & key, const ctrl::Vector6D & error, 
                                               const ctrl::Vector6D & current_vel)
{
  if (m_pd_map.find(key) == m_pd_map.end()) {
    return ctrl::Vector6D::Zero();
  }
  // Perform pd control separately on each Cartesian dimension
  for (int i = 0; i < 6; ++i)  // 3 transition, 3 rotation
  {
    m_cmd(i) = m_pd_map.at(key)[i]->operator()(error[i], current_vel[i]);
  }
  return m_cmd;
}

// ctrl::Vector6D SpatialPDController::operator()(const ctrl::Vector6D & error, const ctrl::Vector6D & current_state,
//                                                const rclcpp::Duration & period)
// {
//   // Perform pd control separately on each Cartesian dimension
//   for (int i = 0; i < 6; ++i)  // 3 transition, 3 rotation
//   {
//     m_cmd(i) = m_pd_controllers[i](error[i], current_state[i], period);
//   }
//   return m_cmd;
// }

bool SpatialPDController::init(rclcpp_lifecycle::LifecycleNode* handle,
                               const std::string & key)
{
  // check if key is already initialized
  if (m_pd_map.count(key) > 0 && !m_pd_map[key].empty())
  { return true;
  }
  // Create pd controllers for each Cartesian dimension
  std::vector<std::string> axes = {".trans_x", ".trans_y", ".trans_z", ".rot_x", ".rot_y", ".rot_z"};
  m_pd_map[key].reserve(6);

  for (int i = 0; i < 6; ++i)  // 3 transition, 3 rotation
  {
    m_pd_map[key].push_back(std::make_unique<PDController>());
    m_pd_map[key][i]->init(key + axes[i], handle);
  }
  return true;
}

}  // namespace cartesian_controller_base
