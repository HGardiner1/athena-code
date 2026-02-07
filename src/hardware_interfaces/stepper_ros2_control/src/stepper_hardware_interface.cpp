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

#include "stepper_ros2_control/stepper_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace stepper_ros2_control
{

hardware_interface::CallbackReturn STEPPERHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  // Parse joint parameters
  for (auto& joint : info_.joints) {
    joint_node_ids.push_back(std::abs(std::stoi(joint.parameters.at("node_id"), nullptr, 0)));
    joint_gear_ratios.push_back(std::abs(std::stoi(joint.parameters.at("gear_ratio"))));
    rated_max.push_back(std::abs(std::stod(joint.parameters.at("rated_max"))) * (M_PI / 180.0)); 

    std::string joint_type = joint.parameters.at("joint_type");
    if (joint_type != "standard" && joint_type != "continuous") {
      RCLCPP_ERROR(
        rclcpp::get_logger("STEPPERHardwareInterface"), 
        "Invalid joint_type parameter for joint %s. Must be 'standard' or 'continuous'.", 
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    joint_type_.push_back(joint_type);
  }

  num_joints = static_cast<int>(info_.joints.size());
  update_rate = std::stoi(info_.hardware_parameters.at("update_rate"));
  can_interface = info_.hardware_parameters.at("can_interface");
  can_command_id = std::stoi(info_.hardware_parameters.at("can_id"), nullptr, 0);
  can_response_id = can_command_id+0x1;

  // Parse hardware parameters with defaults
  if (info_.hardware_parameters.count("update_rate")) {
    update_rate_ = std::stoi(info_.hardware_parameters.at("update_rate"));
  } else {
    update_rate_ = 10;  // Default 10 Hz
  }

  if (info_.hardware_parameters.count("can_interface")) {
    can_interface_ = info_.hardware_parameters.at("can_interface");
  } else {
    can_interface_ = "can0";
  }

  if (info_.hardware_parameters.count("can_id")) {
    can_command_id_ = static_cast<uint32_t>(std::stoul(info_.hardware_parameters.at("can_id"), nullptr, 0));
  } else {
    can_command_id_ = 0x90;  // Default stepper CAN ID
  }
  can_response_id_ = can_command_id_ + 0x1;

  // Initialize state variables
  is_connected_ = 0.0;
  can_connected_ = false;

  // Initialize command and state interface values
  joint_state_position_.assign(num_joints, std::numeric_limits<double>::quiet_NaN());
  joint_state_velocity_.assign(num_joints, 0.0);
  joint_command_position_.assign(num_joints, std::numeric_limits<double>::quiet_NaN());
  joint_command_velocity_.assign(num_joints, 0.0);

  // Motor state storage (per joint)
  motor_position.assign(num_joints, 0.0);
  motor_velocity.assign(num_joints, 0.0);
  device_status.assign(num_joints, 0);

  // Steppers default to velocity control
  control_level_.resize(num_joints, integration_level_t::VELOCITY);
  current_joint_ = 0;
  elapsed_update_time_ = 0.0;

  RCLCPP_INFO(
    rclcpp::get_logger("STEPPERHardwareInterface"),
    "Initialized stepper on CAN interface %s with ID 0x%X (%d joints)", 
    can_interface_.c_str(), can_command_id_, num_joints);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STEPPERHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  RCLCPP_INFO(
    rclcpp::get_logger("STEPPERHardwareInterface"),
    "Configuring stepper hardware...");

  // Open CAN bus
  if (!canBus_.open(can_interface_, 
      std::bind(&STEPPERHardwareInterface::onCanMessage, this, std::placeholders::_1))) 
  {
    RCLCPP_WARN(
      rclcpp::get_logger("STEPPERHardwareInterface"),
      "Failed to open CAN interface %s - running in SIMULATION mode", 
      can_interface_.c_str());
    can_connected_ = false;
  } else {
    can_connected_ = true;
    RCLCPP_INFO(
      rclcpp::get_logger("STEPPERHardwareInterface"),
      "Successfully opened CAN interface %s", can_interface_.c_str());
  }

  is_connected_ = can_connected_ ? 1.0 : 0.0;

  RCLCPP_INFO(
    rclcpp::get_logger("STEPPERHardwareInterface"),
    "Stepper hardware configured (%s)", can_connected_ ? "CAN MODE" : "SIMULATION");

  return hardware_interface::CallbackReturn::SUCCESS;
}

void STEPPERHardwareInterface::onCanMessage(const CANLib::CanFrame& frame) {
  can_rx_frame_ = frame;

  // Check if this is a response from our stepper driver
  if (can_rx_frame_.id != can_response_id_) {
    return;
  }

  // Per protocol spec: response DATA[0] = 0x10 + device_id for motor status
  uint8_t command_byte = can_rx_frame_.data[0];
  uint8_t command_nibble = (command_byte >> 4) & 0x0F;
  uint8_t device_id = command_byte & 0x0F;

  // Only process motor status responses (command nibble = 0x1)
  if (command_nibble != 0x1) {
    return;
  }

  // Find matching joint
  for (int i = 0; i < num_joints; i++) {
    if (device_id == (joint_node_ids[i] & 0x0F)) {
      // Per protocol spec (7 bytes total):
      // DATA[1-3] = Position (24-bit signed, little endian)
      // DATA[4-6] = Velocity (24-bit signed, little endian)
      
      // Decode position (24-bit signed, little endian)
      int32_t raw_position = can_rx_frame_.data[1] | 
                             (can_rx_frame_.data[2] << 8) | 
                             (can_rx_frame_.data[3] << 16);
      // Sign extend 24-bit to 32-bit
      if (raw_position & 0x800000) {
        raw_position |= 0xFF000000;
      }

      // Decode velocity (24-bit signed, little endian)
      int32_t raw_velocity = can_rx_frame_.data[4] | 
                             (can_rx_frame_.data[5] << 8) | 
                             (can_rx_frame_.data[6] << 16);
      // Sign extend 24-bit to 32-bit
      if (raw_velocity & 0x800000) {
        raw_velocity |= 0xFF000000;
      }

      // Convert to joint state
      motor_position[i] = calculate_joint_position_from_motor_position(
        static_cast<double>(raw_position), joint_gear_ratios[i]);
      motor_velocity[i] = calculate_joint_velocity_from_motor_velocity(
        static_cast<double>(raw_velocity), joint_gear_ratios[i]);

      break;  // Found matching joint
    }
  }
}

std::vector<hardware_interface::StateInterface> 
STEPPERHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (int i = 0; i < num_joints; i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_state_position_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_state_velocity_[i]));
  }

  // Add connection status state interface (use first joint name as prefix)
  if (num_joints > 0) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[0].name, "is_connected", &is_connected_));
  }

  RCLCPP_INFO(
    rclcpp::get_logger("STEPPERHardwareInterface"),
    "Exported %zu state interfaces", state_interfaces.size());

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
STEPPERHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (int i = 0; i < num_joints; i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_command_position_[i]));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_command_velocity_[i]));
  }

  RCLCPP_INFO(
    rclcpp::get_logger("STEPPERHardwareInterface"),
    "Exported %zu command interfaces", command_interfaces.size());

  return command_interfaces;
}

hardware_interface::CallbackReturn STEPPERHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("STEPPERHardwareInterface"),
    "Activating stepper hardware...");

  // Initialize command to current state
  joint_command_position_ = joint_state_position_;

  RCLCPP_INFO(
    rclcpp::get_logger("STEPPERHardwareInterface"),
    "Stepper hardware activated");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn STEPPERHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("STEPPERHardwareInterface"),
    "Deactivating stepper hardware...");

  // Per protocol spec: Motor Stop = 0x20 + device_id
  if (can_connected_) {
    for (int i = 0; i < num_joints; i++) {
      can_tx_frame_ = CANLib::CanFrame();
      can_tx_frame_.id = can_command_id_;
      can_tx_frame_.dlc = 1;
      
      uint8_t device_id = joint_node_ids[i] & 0x0F;
      can_tx_frame_.data[0] = 0x20 + device_id;  // Motor Stop Command
      canBus_.send(can_tx_frame_);
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("STEPPERHardwareInterface"),
    "Stepper hardware deactivated%s", can_connected_ ? "" : " (simulated)");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STEPPERHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  RCLCPP_INFO(
    rclcpp::get_logger("STEPPERHardwareInterface"),
    "Cleaning up stepper hardware...");

  // Per protocol spec: Motor Shutdown = 0x30 + device_id
  if (can_connected_) {
    for (int i = 0; i < num_joints; i++) {
      can_tx_frame_ = CANLib::CanFrame();
      can_tx_frame_.id = can_command_id_;
      can_tx_frame_.dlc = 1;

      uint8_t device_id = joint_node_ids[i] & 0x0F;
      can_tx_frame_.data[0] = 0x30 + device_id;  // Motor Shutdown Command
      canBus_.send(can_tx_frame_);
    }

    canBus_.close();
  }

  can_connected_ = false;
  is_connected_ = 0.0;

  RCLCPP_INFO(
    rclcpp::get_logger("STEPPERHardwareInterface"),
    "Stepper hardware cleanup complete");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STEPPERHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{

  RCLCPP_INFO(
    rclcpp::get_logger("STEPPERHardwareInterface"),
    "Shutting down stepper hardware...");

  return on_cleanup(previous_state);
}

double STEPPERHardwareInterface::calculate_joint_position_from_motor_position(
  double motor_position, int gear_ratio)
{
  // Converts from 0.01 deg to radians
  return (motor_position * 0.01 * (M_PI / 180.0)) / gear_ratio;
}

double STEPPERHardwareInterface::calculate_joint_velocity_from_motor_velocity(
  double motor_velocity, int gear_ratio)
{
  // Converts from dps to radians/s
  return (motor_velocity * (M_PI / 180.0)) / gear_ratio;
}

int32_t STEPPERHardwareInterface::calculate_motor_position_from_desired_joint_position(
  double joint_position, int gear_ratio)
{
  // radians -> deg -> 0.01 deg
  return static_cast<int32_t>(std::round((joint_position * (180 / M_PI) * 100) * gear_ratio));
}

int32_t STEPPERHardwareInterface::calculate_motor_velocity_from_desired_joint_velocity(
  double joint_velocity, int gear_ratio)
{
  // radians/s -> deg/s -> 0.01 deg/s
  return static_cast<int32_t>(std::round((joint_velocity * (180 / M_PI) * 100) * gear_ratio));
}

hardware_interface::return_type STEPPERHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  // Round-robin through joints to request status
  current_joint_ = (current_joint_ + 1) % num_joints;
  
  for (int i = 0; i < num_joints; i++) {
    if (current_joint_ == i && can_connected_) {
      can_tx_frame_ = CANLib::CanFrame();
      can_tx_frame_.id = can_command_id_;
      can_tx_frame_.dlc = 1;

      // Per protocol spec: motor_status command = 0x10 + device_id
      uint8_t device_id = joint_node_ids[i] & 0x0F;
      can_tx_frame_.data[0] = 0x10 + device_id;
      canBus_.send(can_tx_frame_);
    }

    // Update joint state from motor state (populated by onCanMessage callback)
    joint_state_velocity_[i] = motor_velocity[i];
    joint_state_position_[i] = motor_position[i];

  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type STEPPERHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  elapsed_update_time_ += period.seconds();
  double update_period = 1.0 / update_rate_;

  // Rate limit CAN messages to configured update rate
  if (elapsed_update_time_ < update_period) {
    return hardware_interface::return_type::OK;
  }
  
  elapsed_update_time_ = 0.0;
  
  for (int i = 0; i < num_joints; i++) {
    if (control_level_[i] == integration_level_t::VELOCITY && 
        std::isfinite(joint_command_velocity_[i])) 
    {
      // Calculate motor velocity from joint command (rad/s -> 0.01 deg/s)
      int32_t speed_control = calculate_motor_velocity_from_desired_joint_velocity(
        std::clamp(joint_command_velocity_[i], -rated_max[i], rated_max[i]), 
        joint_gear_ratios[i]
      );

      if (can_connected_) {
        can_tx_frame_ = CANLib::CanFrame();
        can_tx_frame_.id = can_command_id_;
        can_tx_frame_.dlc = 5;  // Per protocol spec: 5 bytes for speed control

        // ENCODING CAN MESSAGE per protocol spec:
        // DATA[0] = 0x40 + device_id (Speed Closed-Loop Control)
        // DATA[1-4] = speedControl (int32_t, little endian)
        uint8_t device_id = joint_node_ids[i] & 0x0F;
        can_tx_frame_.data[0] = 0x40 + device_id;
        can_tx_frame_.data[1] = static_cast<uint8_t>(speed_control & 0xFF);
        can_tx_frame_.data[2] = static_cast<uint8_t>((speed_control >> 8) & 0xFF);
        can_tx_frame_.data[3] = static_cast<uint8_t>((speed_control >> 16) & 0xFF);
        can_tx_frame_.data[4] = static_cast<uint8_t>((speed_control >> 24) & 0xFF);

        canBus_.send(can_tx_frame_);
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type STEPPERHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string>& start_interfaces,
  const std::vector<std::string>& stop_interfaces)
{
  std::ostringstream ss;
  ss << "perform_command_mode_switch called. start_interfaces: [";
  for (auto &s : start_interfaces) ss << s << ",";
  ss << "] stop_interfaces: [";
  for (auto &s : stop_interfaces) ss << s << ",";
  ss << "]";
  RCLCPP_INFO(rclcpp::get_logger("STEPPERHardwareInterface"), "%s", ss.str().c_str());

  std::vector<integration_level_t> requested_modes(num_joints, integration_level_t::UNDEFINED);

  // Process stop interfaces first
  for (const auto &ifname : stop_interfaces) {
    for (int i = 0; i < num_joints; ++i) {
      const std::string pos_if = info_.joints[i].name + "/" + 
        std::string(hardware_interface::HW_IF_POSITION);
      const std::string vel_if = info_.joints[i].name + "/" + 
        std::string(hardware_interface::HW_IF_VELOCITY);
      if (ifname == pos_if || ifname == vel_if || 
          ifname.find(info_.joints[i].name) != std::string::npos) 
      {
        requested_modes[i] = integration_level_t::UNDEFINED;
      }
    }
  }

  // Process start interfaces
  for (const auto &ifname : start_interfaces) {
    for (int i = 0; i < num_joints; ++i) {
      const std::string pos_if = info_.joints[i].name + "/" + 
        std::string(hardware_interface::HW_IF_POSITION);
      const std::string vel_if = info_.joints[i].name + "/" + 
        std::string(hardware_interface::HW_IF_VELOCITY);
      if (ifname == pos_if) {
        requested_modes[i] = integration_level_t::POSITION;
      } else if (ifname == vel_if) {
        requested_modes[i] = integration_level_t::VELOCITY;
      }
    }
  }

  // Apply requested modes
  for (int i = 0; i < num_joints; ++i) {
    if (requested_modes[i] == integration_level_t::UNDEFINED) {
      bool was_stopped = false;
      for (const auto &ifname : stop_interfaces) {
        if (ifname.find(info_.joints[i].name) != std::string::npos) {
          was_stopped = true;
          break;
        }
      }
      if (was_stopped) {
        control_level_[i] = integration_level_t::UNDEFINED;
        joint_command_velocity_[i] = 0;
        joint_command_position_[i] = joint_state_position_[i];
        RCLCPP_INFO(
          rclcpp::get_logger("STEPPERHardwareInterface"),
          "Joint %s: stopped -> set UNDEFINED", info_.joints[i].name.c_str());
      }
    } else {
      control_level_[i] = requested_modes[i];
      if (requested_modes[i] == integration_level_t::VELOCITY) {
        joint_command_velocity_[i] = 0;
        RCLCPP_INFO(
          rclcpp::get_logger("STEPPERHardwareInterface"),
          "Joint %s: switched to VELOCITY", info_.joints[i].name.c_str());
      } else if (requested_modes[i] == integration_level_t::POSITION) {
        joint_command_position_[i] = joint_state_position_[i];
        RCLCPP_INFO(
          rclcpp::get_logger("STEPPERHardwareInterface"),
          "Joint %s: switched to POSITION", info_.joints[i].name.c_str());
      }
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace stepper_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  stepper_ros2_control::STEPPERHardwareInterface,
  hardware_interface::SystemInterface)
