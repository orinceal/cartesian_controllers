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
/*!\file    PDController.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2019/10/16
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_controller_base/PDController.h>

#include <utility>

namespace cartesian_controller_base
{
PDController::PDController() {}

PDController::~PDController() {}

void PDController::init(const std::string & params,
                        rclcpp_lifecycle::LifecycleNode* handle)
{
  m_params = params;
  m_handle = handle->shared_from_this();

  auto auto_declare = [handle](const std::string & name) -> double
  {
    if (!handle->has_parameter(name))
    {
      handle->declare_parameter(name, 0.0);
    }
    auto param = handle->get_parameter(name);

    if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      return static_cast<double>(param.as_int());
    }
    return param.as_double();
    // return handle->get_parameter(name).as_double();
  };

  m_p = auto_declare(m_params + ".p");
  m_d = auto_declare(m_params + ".d");

  // parameter modification callback
  m_callback_handle = handle->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      for (const auto & param : parameters) {
        if (param.get_name() == m_params + ".p") {
          m_p = (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                ? static_cast<double>(param.as_int()) : param.as_double();
        }
        if (param.get_name() == m_params + ".d") {
          m_d = (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                ? static_cast<double>(param.as_int()) : param.as_double();
        }
      }
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      return result;
    }
  );
}

double PDController::operator()(const double & error, const double & damping_term)
{
  // // Get latest gains
  // m_handle->get_parameter(m_params + ".p", m_p);
  // m_handle->get_parameter(m_params + ".d", m_d);
  // double result = m_p * error + m_d * (error - m_last_p_error) / period.seconds();
  // m_last_p_error = error;

  // apply p_gain on error and d_gain on actual velocity from forward kinematics
  return (m_p * error) - (m_d * damping_term);
}

}  // namespace cartesian_controller_base
