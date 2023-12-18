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

#include "igus_axis_driver/singlebot.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace igus_axis_driver
{
hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // on init is for initialising data
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  axisName=info_.hardware_parameters["axisName"];
  ip=(info_.hardware_parameters["ip"]);

  port=stoi(info_.hardware_parameters["port"]);
  acc=stod(info_.hardware_parameters["acc"]);
  dec=stod(info_.hardware_parameters["dec"]);
  setvel=stod(info_.hardware_parameters["setvel"]);

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{


  xAxis.startConnection(ip, port);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RRBotSystemPositionOnlyHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
    axisName, hardware_interface::HW_IF_POSITION, &pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
    axisName, hardware_interface::HW_IF_VELOCITY, &setvel));


  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RRBotSystemPositionOnlyHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
    axisName, hardware_interface::HW_IF_VELOCITY, &setvel));


    command_interfaces.emplace_back(hardware_interface::CommandInterface(
    axisName, hardware_interface::HW_IF_POSITION, &pos));

  return command_interfaces;
}

hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Activating ...please wait...");

	xAxis.setDebugModeON();
	xAxis.setDebugModeOFF();

	// Run through State Machine --> Current is applied to the motor
	xAxis.runStateMachine();



	// Start Homing only if not yet referenced
	if (xAxis.readObjectValue(0x20, 0x14, 0) == 0) 	// Read the value of the Object 2014.1 "Status Flags" 

	{
	xAxis.homing(50, 1, 100); // Homing (Switch search speed[�/s], Zero search speed[�/s], Acceleration for Homing[�/s�])
	}

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
	xAxis.waitForReady();
	xAxis.setShutdown();

	// Gracefully close everything down
	
	close(xAxis.sock);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RRBotSystemPositionOnlyHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  pos=xAxis.getCurrentPosInMM()/1000;
  setvel=xAxis.getCurrentVelInMMS()/1000; // ROS Units are meters

  std::cout << "position in meters" << pos << std::endl;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotSystemPositionOnlyHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // xAxis.profilePositionAbs(pos*1000, setvel, acc, dec);
  // xAxis.profilePositionAbs_Async(pos*1000, setvel, acc, dec);
    std::cout << "sending veocity command:" << setvel*1000 << std::endl;

  xAxis.profileVelocity(setvel*1000,acc);
  return hardware_interface::return_type::OK;
}

}  // namespace igus_axis_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  igus_axis_driver::RRBotSystemPositionOnlyHardware, hardware_interface::SystemInterface)
