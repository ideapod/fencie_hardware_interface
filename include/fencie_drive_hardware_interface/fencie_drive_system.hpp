// Copyright 2021 ros2_control Development Team
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

#ifndef FENCIE_DRIVE_HARDWARE_INTERFACE__FENCIE_DRIVE_SYSTEM_HPP_
#define FENCIE_DRIVE_HARDWARE_INTERFACE__FENCIE_DRIVE_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "quadmotor_interfaces/msg/quad_motor_encoder_counts.hpp"
#include "quadmotor_interfaces/msg/quad_motor_velocities.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fencie_drive_hardware_interface
{
class FencieDriveSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FencieDriveSystemHardware)

  /*
   * expects parameters:
   * quad_velocity_topic
   * quad_encoder_topic
   */
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  // Parameters for the DiffBot simulation
  std::string velocity_topic_name_;
  std::string encoder_topic_name_;

  rclcpp::Subscription<quadmotor_interfaces::msg::QuadMotorEncoderCounts>::SharedPtr encoder_count_subscriber_;
  rclcpp::Publisher<quadmotor_interfaces::msg::QuadMotorVelocities>::SharedPtr motor_velocity_publisher_;
  rclcpp::Node::SharedPtr node_;
  quadmotor_interfaces::msg::QuadMotorEncoderCounts latestEncoderCounts_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  // std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
};

}  // namespace fencie_drive_hardware_interface

#endif  // FENCIE_DRIVE_HARDWARE_INTERFACE__FENCIE_DRIVE_SYSTEM_HPP_
