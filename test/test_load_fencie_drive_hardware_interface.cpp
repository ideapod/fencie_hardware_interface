// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#include <gmock/gmock.h>

#include <string>

#include "hardware_interface/resource_manager.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

class TestFencieDriveHardwareInterface : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // TODO(anyone): Extend this description to your robot
    fencie_hardware_interface_2dof_ =
      R"(
        <ros2_control name="fencie_drive_system_interface" type="system">
          <hardware>
            <plugin>fencie_drive_hardware_interface/FencieDriveSystemHardware</plugin>
            <param name="quad_velocity_topic">/quad_motor_velocities</param>
            <param name="quad_encoder_topic">/quad_motor_encoder</param>
          </hardware>
          <joint name="lhf">
            <command_interface name="velocity">
                <param name="min">0</param>
                <param name="max">10</param>
            </command_interface>
            <state_interface name="velocity"/>
          </joint>
          <joint name="rhf">
            <command_interface name="velocity">
                <param name="min">0</param>
                <param name="max">10</param>
            </command_interface>
            <state_interface name="velocity"/>
          </joint>
      </ros2_control>
    )";
  }

  std::string fencie_hardware_interface_2dof_;
};

TEST_F(TestFencieDriveHardwareInterface, load_fencie_hardware_interface_2dof)
{
  auto urdf = ros2_control_test_assets::urdf_head + fencie_hardware_interface_2dof_ +
              ros2_control_test_assets::urdf_tail;
  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf));
}
