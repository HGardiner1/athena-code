// Copyright (c) 2024 UMD Loop
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
//

#ifndef STEPPER_HARDWARE_INTERFACE_HPP_
#define STEPPER_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <cstdint>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Real-time CAN communication library
#include <array>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include "umdloop_can_library/SocketCanBus.hpp"
#include "umdloop_can_library/CanFrame.hpp"

namespace stepper_ros2_control
{
class STEPPERHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(STEPPERHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces
  ) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Helper Functions

  // CAN callback for receiving messages
  void onCanMessage(const CANLib::CanFrame& frame);
  void logger_function();

private:

  int update_rate;
  double elapsed_update_time; // Time since last hardware interface update
  double elapsed_time; // Time since first hardware interface update
  double elapsed_logger_time; // Time since last logger update
  int logger_rate; // Logger update rate
  int logger_state; // Logger on/off state

  int num_joints;
  int current_joint_;

  std::string can_interface;

  uint32_t can_command_id;
  uint32_t can_response_id;

  // Store the state for the robot
  std::vector<double> joint_state_position_;
  std::vector<double> joint_state_velocity_;
  
  // Store the command for the robot
  std::vector<double> joint_command_position_;
  std::vector<double> joint_command_velocity_;

  // Motor state from CAN (per joint)
  std::vector<double> motor_position;
  std::vector<double> motor_velocity;
  std::vector<double> rated_max;
  std::vector<int> device_status;

  // CAN communication
  CANLib::SocketCanBus canBus_;
  CANLib::CanFrame can_tx_frame_;
  CANLib::CanFrame can_rx_frame_;

  bool can_connected_;
  double is_connected_;  // State interface for connection status

  // Joint parameters
  std::vector<int> joint_node_ids;
  std::vector<int> joint_gear_ratios;
  std::vector<std::string> joint_type_;

  enum class integration_level_t : std::uint8_t
  {
    UNDEFINED = 0,
    POSITION = 1,
    VELOCITY = 2,
  };
  
  // CAN Commands
  static constexpr uint8_t PCB_HEARTBEAT_CMD = 0X10;
  static constexpr uint8_t LED_STATUS_CMD = 0x11;
  static constexpr uint8_t ABSOLUTE_POS_CONTROL_CMD = 0x20;
  static constexpr uint8_t VELOCITY_CONTROL_CMD = 0x30;
  static constexpr uint8_t MOTOR_STATE_CMD = 0x40;
  static constexpr uint8_t MOTOR_STATUS_CMD = 0x50;
  static constexpr uint8_t MAINTENANCE_CMD = 0x60;
  static constexpr uint8_t SERVO_SPECS_CMD = 0x70;

  // Active control mode for each actuator
  std::vector<integration_level_t> control_level_;
};

}  // namespace stepper_ros2_control

#endif  // STEPPER_HARDWARE_INTERFACE_HPP_