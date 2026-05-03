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

void STEPPERHardwareInterface::logger_function()
{
  // Prevents breaking the logger
  if (num_joints == 0) return;

  // Building Message
  std::string log_msg = "\033[2J\033[H \nSTEPPER Logger";
  std::ostringstream oss;
  std::string status;

  // HWI Specific
  oss << "\n--- HWI Specific ---\n"
      << "CAN Interface: " << can_interface
      << " | Command CAN ID: 0x" << std::hex << std::uppercase << can_command_id
      << " | Response CAN ID: 0x" << std::hex << std::uppercase << can_response_id
      << " | HWI Update Rate: " << update_rate
      << " | Logger Update Rate: " << logger_rate << "\n"
      << "Elapsed Time since first update: " << elapsed_time << "\n"
      << "\n--- Joint Specific ---";

  for (int i = 0; i < num_joints; i++) {
    switch (device_status[i]) {
      case 0x00: status = "IDLE (ready)";      break;
      case 0x01: status = "ACTIVE (busy)";     break;
      case 0x02: status = "DOES NOT EXIST";    break;
      case 0x03: status = "ERROR";             break;
      default:   status = "UNDEFINED";         break;
    }

    oss << "\nJOINT: " << info_.joints[i].name << "\n"
        << "Parameters: Node ID: 0x" << std::hex << std::uppercase << joint_node_ids[i]
        << " | Gear Ratio: " << joint_gear_ratios[i]
        << " | Device Status: " << std::hex << std::uppercase << device_status[i]
        << " - " << status << "\n"
        << "-- Commands --\n"
        << "Control Mode: " << static_cast<int>(control_level_[i]) << "\n"
        << "Motor Position: " << motor_position[i]
        << " | Joint Command Position: " << joint_command_position_[i] << "\n"
        << "Motor Velocity: " << motor_velocity[i]
        << " | Joint Command Velocity: " << joint_command_velocity_[i] << "\n"
        << "-- State --\n"
        << "Joint Position: " << joint_state_position_[i]
        << " | Joint Velocity: " << joint_state_velocity_[i] << "\n";
  }

  log_msg += oss.str();
  RCLCPP_INFO(rclcpp::get_logger("STEPPERHardwareInterface"), log_msg.c_str());
}

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
    joint_node_ids.push_back(std::clamp(std::stoi(joint.parameters.at("node_id"), nullptr, 0), 0x0, 0xF));
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

  // Parse hardware parameters with defaults
  if (info_.hardware_parameters.count("update_rate")) {
    update_rate = std::stoi(info_.hardware_parameters.at("update_rate"));
  } else {
    update_rate = 10;
  }

  if (info_.hardware_parameters.count("logger_rate")) {
    logger_rate = std::stoi(info_.hardware_parameters.at("logger_rate"));
  } else {
    logger_rate = 5;
  }

  if (info_.hardware_parameters.count("logger_state")) {
    logger_state = std::stoi(info_.hardware_parameters.at("logger_state"));
  } else {
    logger_state = 0;
  }

  if (info_.hardware_parameters.count("can_interface")) {
    can_interface = info_.hardware_parameters.at("can_interface");
  } else {
    can_interface = "can0";
  }

  if (info_.hardware_parameters.count("can_id")) {
    can_command_id = static_cast<uint32_t>(std::stoul(info_.hardware_parameters.at("can_id"), nullptr, 0));
  } else {
    can_command_id = 0x90;
  }
  can_response_id = can_command_id + 0x1;

  // Initialize timing
  elapsed_update_time = 0.0;
  elapsed_time        = 0.0;
  elapsed_logger_time = 0.0;
  current_joint_      = 0;

  // Initialize state variables
  is_connected_  = 0.0;
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

  RCLCPP_INFO(
    rclcpp::get_logger("STEPPERHardwareInterface"),
    "Initialized stepper on CAN interface %s with ID 0x%X (%d joints)",
    can_interface.c_str(), can_command_id, num_joints);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn STEPPERHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return this->on_cleanup(previous_state);
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

hardware_interface::CallbackReturn STEPPERHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("STEPPERHardwareInterface"),
    "Configuring stepper hardware...");

  if (!canBus_.open(can_interface,
      std::bind(&STEPPERHardwareInterface::onCanMessage, this, std::placeholders::_1)))
  {
    RCLCPP_WARN(
      rclcpp::get_logger("STEPPERHardwareInterface"),
      "Failed to open CAN interface %s - running in SIMULATION mode",
      can_interface.c_str());
    can_connected_ = false;
  } else {
    can_connected_ = true;
    RCLCPP_INFO(
      rclcpp::get_logger("STEPPERHardwareInterface"),
      "Successfully opened CAN interface %s", can_interface.c_str());
  }

  is_connected_ = can_connected_ ? 1.0 : 0.0;

  RCLCPP_INFO(
    rclcpp::get_logger("STEPPERHardwareInterface"),
    "Stepper hardware configured (%s)", can_connected_ ? "CAN MODE" : "SIMULATION");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Per protocol spec, response decoding uses 16-bit values (not 24-bit).
// All responses return: data[1-2] = position (int16, deg), data[3-4] = velocity (int16, deg/s).
void STEPPERHardwareInterface::onCanMessage(const CANLib::CanFrame& frame)
{
  // RCLCPP_INFO(
  //   rclcpp::get_logger("STEPPER"),
  //   "RX id=0x%X dlc=%d b0=0x%02X",
  //   frame.id, frame.dlc, frame.data[0]);

  can_rx_frame_ = frame;

  if (can_rx_frame_.id != can_response_id) {
    return;
  }

  uint8_t command_nibble  = (can_rx_frame_.data[0] >> 4) & 0x0F;
  uint8_t device_id_nibble = can_rx_frame_.data[0] & 0x0F;

  for (int i = 0; i < num_joints; i++) {
    if (device_id_nibble != static_cast<uint8_t>(joint_node_ids[i] & 0x0F)) {
      continue;
    }

    if (command_nibble == MOTOR_STATE_CMD) {
      // Per protocol spec, position and velocity are 16-bit signed (deg, deg/s), little endian
      int16_t raw_position = static_cast<int16_t>(
          can_rx_frame_.data[1] | (can_rx_frame_.data[2] << 8));
      int16_t raw_velocity = static_cast<int16_t>(
          can_rx_frame_.data[3] | (can_rx_frame_.data[4] << 8));

      // Convert deg -> rad and apply gear ratio
      motor_position[i] = (static_cast<double>(raw_position) * M_PI / 180.0) / joint_gear_ratios[i];
      motor_velocity[i] = (static_cast<double>(raw_velocity) * M_PI / 180.0) / joint_gear_ratios[i];

    } else if (command_nibble == MOTOR_STATUS_CMD) {
      device_status[i] = can_rx_frame_.data[1];
    }

    break;  // Matched joint, no need to keep iterating
  }
}

hardware_interface::CallbackReturn STEPPERHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("STEPPERHardwareInterface"),
    "Activating ...please wait...");

  joint_command_position_ = joint_state_position_;

  for (size_t i = 0; i < joint_command_position_.size(); ++i) {
    RCLCPP_INFO(
      rclcpp::get_logger("STEPPERHardwareInterface"),
      "Joint %zu command position initialized to: %f", i, joint_command_position_[i]);
  }

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

  if (can_connected_) {
    for (int i = 0; i < num_joints; i++) {
      can_tx_frame_     = CANLib::CanFrame();
      can_tx_frame_.id  = can_command_id;
      can_tx_frame_.dlc = 2;

      // (MAINTENANCE_CMD << 4) | port_id = 0x60 | port_id, maintenance cmd 1 = Stop stepper
      uint8_t device_id_nibble = joint_node_ids[i] & 0x0F;
      can_tx_frame_.data[0] = static_cast<uint8_t>((MAINTENANCE_CMD << 4) | device_id_nibble);
      can_tx_frame_.data[1] = 1;  // Maintenance cmd 1 = Stop stepper
      canBus_.send(can_tx_frame_);
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("STEPPERHardwareInterface"),
    "Stepper hardware deactivated%s", can_connected_ ? "" : " (simulated)");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// can_connected_ and is_connected_ are always reset regardless of whether CAN was open.
hardware_interface::CallbackReturn STEPPERHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("STEPPERHardwareInterface"),
    "Cleaning up stepper hardware...");

  if (can_connected_) {
    for (int i = 0; i < num_joints; i++) {
      can_tx_frame_     = CANLib::CanFrame();
      can_tx_frame_.id  = can_command_id;
      can_tx_frame_.dlc = 2;

      // FIX #4: Assign device_id_nibble (was previously using uninitialized variable)
      uint8_t device_id_nibble = joint_node_ids[i] & 0x0F;
      can_tx_frame_.data[0] = static_cast<uint8_t>((MAINTENANCE_CMD << 4) | device_id_nibble);
      can_tx_frame_.data[1] = 2;  // Maintenance cmd 2 = Shutdown stepper
      canBus_.send(can_tx_frame_);
    }

    canBus_.close();
  }

  // Always reset connection state
  can_connected_ = false;
  is_connected_  = 0.0;

  RCLCPP_INFO(
    rclcpp::get_logger("STEPPERHardwareInterface"),
    "Stepper hardware cleanup complete");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type STEPPERHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (can_connected_) {
    current_joint_ = (current_joint_ + 1) % num_joints;

    can_tx_frame_     = CANLib::CanFrame();
    can_tx_frame_.id  = can_command_id;
    can_tx_frame_.dlc = 2;

    // MOTOR_STATE_CMD = 0x4 → (0x4 << 4) | port_id = 0x40 | port_id
    uint8_t port_id = joint_node_ids[current_joint_] & 0x0F;
    can_tx_frame_.data[0] = static_cast<uint8_t>((MOTOR_STATE_CMD << 4) | port_id);
    can_tx_frame_.data[1] = 0x01;  // Validate the request
    canBus_.send(can_tx_frame_);
  }

  // Copy motor state (updated asynchronously by onCanMessage) into joint state
  for (int i = 0; i < num_joints; i++) {
    joint_state_position_[i] = motor_position[i];
    joint_state_velocity_[i] = motor_velocity[i];

    // Return error on any fault status
    if (device_status[i] != 0x00 && device_status[i] != 0x01 && device_status[i] != -1) {
      RCLCPP_ERROR(
        rclcpp::get_logger("STEPPERHardwareInterface"),
        "Joint %s has fault device_status=0x%02X", info_.joints[i].name.c_str(), device_status[i]);
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type STEPPERHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  elapsed_update_time += period.seconds();
  double update_period = 1.0 / update_rate;

  // Rate-limit CAN messages to configured update rate
  if (elapsed_update_time < update_period) {
    return hardware_interface::return_type::OK;
  }

  elapsed_update_time = 0.0;
  elapsed_time += period.seconds();

  for (int i = 0; i < num_joints; i++) {
    if (control_level_[i] == integration_level_t::VELOCITY &&
        std::isfinite(joint_command_velocity_[i]))
    {
      // Stepper only accepts three discrete speeds: +900, -900, or 0 (deg/s).
      // Map the signed velocity command to the nearest valid value.
      int16_t speed_dps;
      if (joint_command_velocity_[i] > 0.0) {
        speed_dps = 900;
      } else if (joint_command_velocity_[i] < 0.0) {
        speed_dps = -900;
      } else {
        speed_dps = 0;
      }

      if (can_connected_) {
        can_tx_frame_     = CANLib::CanFrame();
        can_tx_frame_.id  = can_command_id;
        can_tx_frame_.dlc = 3;  // 3 bytes per spec

        // VELOCITY_CONTROL_CMD = 0x3 → (0x3 << 4) | port_id = 0x30 | port_id
        uint8_t port_id = joint_node_ids[i] & 0x0F;
        can_tx_frame_.data[0] = static_cast<uint8_t>((VELOCITY_CONTROL_CMD & 0xF0) | port_id);
        can_tx_frame_.data[1] = static_cast<uint8_t>(speed_dps & 0xFF);         // low byte
        can_tx_frame_.data[2] = static_cast<uint8_t>((speed_dps >> 8) & 0xFF);  // high byte

        RCLCPP_INFO(rclcpp::get_logger("STEPPERHardwareInterface"), "Data[0]: 0x%02X | Data[1-2] (speed_dps): %d | Joint: %s",
          can_tx_frame_.data[0], speed_dps, info_.joints[i].name.c_str());

        canBus_.send(can_tx_frame_);
      }
    }
    else if (control_level_[i] == integration_level_t::POSITION &&
             std::isfinite(joint_command_position_[i]))
    {
      // Convert rad -> deg, apply gear ratio, clamp to int16 range
      int16_t position_deg = static_cast<int16_t>(std::clamp(
        joint_command_position_[i] * (180.0 / M_PI) * joint_gear_ratios[i],
        static_cast<double>(std::numeric_limits<int16_t>::min()),
        static_cast<double>(std::numeric_limits<int16_t>::max())
      ));

      if (can_connected_) {
        can_tx_frame_     = CANLib::CanFrame();
        can_tx_frame_.id  = can_command_id;
        can_tx_frame_.dlc = 3;

        // ABSOLUTE_POS_CONTROL_CMD = 0x2 → (0x2 << 4) | port_id = 0x20 | port_id
        uint8_t port_id = joint_node_ids[i] & 0x0F;
        can_tx_frame_.data[0] = static_cast<uint8_t>((ABSOLUTE_POS_CONTROL_CMD << 4) | port_id);
        can_tx_frame_.data[1] = static_cast<uint8_t>(position_deg & 0xFF);         // low byte
        can_tx_frame_.data[2] = static_cast<uint8_t>((position_deg >> 8) & 0xFF);  // high byte

        canBus_.send(can_tx_frame_);
      }
    }
  }

  // Optional logger
  if (logger_state == 1) {
    elapsed_logger_time += period.seconds();
    double logger_period = 1.0 / logger_rate;
    if (elapsed_logger_time >= logger_period) {
      elapsed_logger_time = 0.0;
      logger_function();
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
        control_level_[i]          = integration_level_t::UNDEFINED;
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