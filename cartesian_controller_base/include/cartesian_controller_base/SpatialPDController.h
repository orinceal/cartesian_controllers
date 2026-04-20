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
/*!\file    SpatialPDController.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/28
 *
 */
//-----------------------------------------------------------------------------

#ifndef SPATIAL_PD_CONTROLLER_H_INCLUDED
#define SPATIAL_PD_CONTROLLER_H_INCLUDED
#include <memory>
#include <cartesian_controller_base/PDController.h>
#include <cartesian_controller_base/Utility.h>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace cartesian_controller_base
{
/**
 * @brief A 6-dimensional PD controller class
 *
 * This class implements separate PD controllers for each of the Cartesian
 * axes, i.e. three translational controllers and three rotational controllers.
 */
class SpatialPDController
{
public:
  SpatialPDController();

  bool init(rclcpp_lifecycle::LifecycleNode* handle, const std::string & key);

  /**
     * @brief Call operator for one control cycle
     *
     * @param error The control error to reduce. Target - current.
     * @param period The period for this control step.
     * @param current_vel task-space velocity x_dot to apply damping to
     *
     * @return The controlled 6-dim vector (translational, rotational).
     */
  ctrl::Vector6D operator()(const std::string & key, const ctrl::Vector6D & error);
  ctrl::Vector6D operator()(const std::string & key, const ctrl::Vector6D & error, const ctrl::Vector6D & current_vel);

private:
  ctrl::Vector6D m_cmd;
  std::map<std::string, std::vector<std::unique_ptr<PDController>>> m_pd_map;
  // std::vector<PDController> m_pd_controllers;
};

}  // namespace cartesian_controller_base

#endif
