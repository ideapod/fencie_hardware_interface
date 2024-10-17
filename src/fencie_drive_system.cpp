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

#include "fencie_drive_hardware_interface/fencie_drive_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fencie_drive_hardware_interface
{
  
  /* set up all the joint states and get the ros node */
  hardware_interface::CallbackReturn FencieDriveSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
  {
    RCLCPP_INFO(
    rclcpp::get_logger("FencieDriveSystemHardware"),
    "Entering init state."
    );
    // allow super to initialise with hardware info
    if (
      hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // start node here? 

    // example getting parameter from URDF: 
    // hw_start_sec_ = hardware_interface::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
    velocity_topic_name_ = info_.hardware_parameters["quad_velocity_topic"]; //, "/quad_motor_velocities");
    encoder_topic_name_ = info_.hardware_parameters["quad_encoder_topic"]; // , "/quad_motor_encoder");

    RCLCPP_INFO(
    rclcpp::get_logger("FencieDriveSystemHardware"),
      "got topics: velocity: %s, encoder: %s",
      velocity_topic_name_.c_str(),
      encoder_topic_name_.c_str()
    );
    // size the positions, velocities, commands vectors to the numbers in the config. 
    // hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    // enumerate the joints 
    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
      // DiffBotSystem validated the exact number of interfaces were specified against each joint. 
      // Ha! not happening - let's just enumerate. 
      RCLCPP_INFO(
        rclcpp::get_logger("FencieDriveSystemHardware"),
        "Joint '%s' has %zu command interfaces found, %zu state interfaces", 
        joint.name.c_str(),
        joint.command_interfaces.size(),
        joint.state_interfaces.size()
      );

      // check name of interface
      // if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

// establish comms
hardware_interface::CallbackReturn FencieDriveSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("FencieDriveSystemHardware"),
    "Entering config state."
  ); 
  // rrbot_comms_ = std::make_unique<dr_denis_rrbot_comms::DrDenisRRBotComms>(info_.joints.size());
  // TODO - subscribe to motor encoder messages

  // set up our local state.
  // get the node 
  rclcpp::NodeOptions options;
  options.arguments({ "--ros-args", "-r", "__node:=topic_hardware_interface_" + info_.name });
  node_ = rclcpp::Node::make_shared("_", options);
  
  RCLCPP_INFO(
  rclcpp::get_logger("FencieDriveSystemHardware"),
  "started node."
  );
  
  // publish motor velocities
  motor_velocity_publisher_ = 
    node_->create_publisher<quadmotor_interfaces::msg::QuadMotorVelocities>(
      velocity_topic_name_, rclcpp::QoS(1));

  // subscribe to encoder messages  
  encoder_count_subscriber_ = 
    node_->create_subscription<quadmotor_interfaces::msg::QuadMotorEncoderCounts>(
      encoder_topic_name_, rclcpp::SensorDataQoS(),
      [this](const quadmotor_interfaces::msg::QuadMotorEncoderCounts::SharedPtr in_counts) { 
        latestEncoderCounts_ = *in_counts; 
      });  

  return CallbackReturn::SUCCESS;
}

// release everything so we can start again. 
hardware_interface::CallbackReturn FencieDriveSystemHardware::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("FencieDriveSystemHardware"),
    "Entering shutdown state."
  );
  node_.reset(); // release our pointer to the node. (hopefully it goes away)

  return CallbackReturn::SUCCESS;
}

// terminate comms
hardware_interface::CallbackReturn FencieDriveSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("FencieDriveSystemHardware"),
    "Entering cleanup state."
  );

  // release our publisher and subscription.
  encoder_count_subscriber_.reset(); 
  motor_velocity_publisher_.reset();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FencieDriveSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    //   info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FencieDriveSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn FencieDriveSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("FencieDriveSystemHardware"), "Activating");

  // set some default values
  for (auto i = 0u; i < hw_velocities_.size(); i++)
  {
    if (std::isnan(hw_velocities_[i]))
    {
      // hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("FencieDriveSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FencieDriveSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("FencieDriveSystemHardware"), "Deactivating ...please wait...");

  

  RCLCPP_INFO(rclcpp::get_logger("FencieDriveSystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FencieDriveSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period */)
{
  // copy from the encoder values to the velocities
  // the encoder values are written from the subscription to the encoder messages. (see lambda above)

  // encoder represents rotations - how many rotations of the ungeared motor.
  // we work out how many ungeared rotations represent 1 shaft rotation.
  // then work out how many shaft rotations per second. 
  hw_velocities_[0] = (double )latestEncoderCounts_.motor1_encoder_count;
  hw_velocities_[1] = (double )latestEncoderCounts_.motor2_encoder_count;
  hw_velocities_[2] = (double )latestEncoderCounts_.motor3_encoder_count;
  hw_velocities_[3] = (double )latestEncoderCounts_.motor4_encoder_count;
  for (std::size_t i = 0; i < hw_velocities_.size(); i++)
  {
    // hw_positions_[i] = hw_positions_[i] + period.seconds() * hw_velocities_[i];
    RCLCPP_DEBUG(
      rclcpp::get_logger("FencieDriveSystemHardware"),
      "Got velocity state %.5f for '%s'!", hw_velocities_[i], 
      info_.joints[i].name.c_str()
    );
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type fencie_drive_hardware_interface ::FencieDriveSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_DEBUG(rclcpp::get_logger("FencieDriveSystemHardware"), "Writing...");

  for (auto i = 0u; i < hw_commands_.size(); i++)
  {
    // Simulate sending commands to the hardware
    RCLCPP_DEBUG(
      rclcpp::get_logger("FencieDriveSystemHardware"), "Got command %.5f for '%s'!", hw_commands_[i],
      info_.joints[i].name.c_str());
  }

  // make up the message and then publish it.
  auto message = quadmotor_interfaces::msg::QuadMotorVelocities();                                   
  message.motor1_velocity = hw_commands_[0];
  message.motor2_velocity = hw_commands_[1];
  message.motor3_velocity = hw_commands_[2];
  message.motor4_velocity = hw_commands_[3];
  
  RCLCPP_DEBUG(
    rclcpp::get_logger("FencieDriveSystemHardware"), 
    "Publishing: M1= %u, M2=%u, M3=%u, M4=%u'",
    message.motor1_velocity,
    message.motor2_velocity,
    message.motor3_velocity,
    message.motor4_velocity
    );
    motor_velocity_publisher_->publish(message);

  RCLCPP_DEBUG(rclcpp::get_logger("FencieDriveSystemHardware"), "Joints successfully written!");

  return hardware_interface::return_type::OK;
}

}  // namespace fencie_drive_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  fencie_drive_hardware_interface::FencieDriveSystemHardware, hardware_interface::SystemInterface)
