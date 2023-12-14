// Copyright 2020 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef igus_axis_driver__RRBOT_HPP_
#define igus_axis_driver__RRBOT_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "igus_axis_driver/visibility_control.h"
#include "D1.h"
#include "igus_axis_driver/D1.h"

namespace igus_axis_driver
{
class RRBotSystemPositionOnlyHardware : public hardware_interface::SystemInterface
{

struct AxisConfig
{
    std::string axisName="";
    std::string ip=0;
    int port=0;
    int min=0;
    int max=0;
    double pos=0;
    double curvel=0;
    double setvel=0;

    double acc=0;
    double dec=0;

    
};
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RRBotSystemPositionOnlyHardware);

  igus_axis_driver_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  igus_axis_driver_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  igus_axis_driver_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  igus_axis_driver_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  igus_axis_driver_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  igus_axis_driver_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  igus_axis_driver_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  igus_axis_driver_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters for the RRBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  D1 xAxis;
  std::string axisName;
  std::string ip;
  int port;
  int min;
  int max;
  double pos;
  double setpos;
  
  double curvel;
  double setvel;

  double acc;
  double dec;
  // AxisConfig AxisCfg;
};

}  // namespace igus_axis_driver

#endif  // igus_axis_driver__RRBOT_HPP_
