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

#include "laser_ros2_control/laser_hardware_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace laser_ros2_control
{

hardware_interface::CallbackReturn LaserHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  laser_state_ = 0.0;
  is_connected_ = 0.0;
  laser_command_ = 0.0;
  can_connected_ = false;

  // Parse hardware parameters
  if (info_.hardware_parameters.count("can_interface")) {
    can_interface_ = info_.hardware_parameters.at("can_interface");
  } else {
    can_interface_ = "can0";
  }

  if (info_.hardware_parameters.count("can_id")) {
    can_id_ = static_cast<uint32_t>(
      std::stoul(info_.hardware_parameters.at("can_id"), nullptr, 0));
  } else {
    can_id_ = 0x130;
  }

  if (info_.hardware_parameters.count("port_id")) {
    port_id_ = static_cast<uint8_t>(
      std::stoul(info_.hardware_parameters.at("port_id"), nullptr, 0));
  } else {
    port_id_ = 0;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Initialized laser on CAN interface %s with ID 0x%X and port ID %u",
    can_interface_.c_str(), can_id_, port_id_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Configuring laser hardware...");

  if (can_connected_) {
    canBus_.close();
    can_connected_ = false;
  }

  if (!canBus_.open(
        can_interface_,
        std::bind(&LaserHardwareInterface::onCanMessage, this, std::placeholders::_1)))
  {
    RCLCPP_WARN(
      rclcpp::get_logger("LaserHardwareInterface"),
      "Failed to open CAN interface %s - running in SIMULATION mode",
      can_interface_.c_str());
    can_connected_ = false;
  } else {
    can_connected_ = true;
    RCLCPP_INFO(
      rclcpp::get_logger("LaserHardwareInterface"),
      "Successfully opened CAN interface %s", can_interface_.c_str());
  }

  is_connected_ = can_connected_ ? 1.0 : 0.0;

  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Laser hardware configured (%s)", can_connected_ ? "CAN MODE" : "SIMULATION");

  return hardware_interface::CallbackReturn::SUCCESS;
}

void LaserHardwareInterface::onCanMessage(const CANLib::CanFrame & frame)
{
  if (frame.id != can_id_ || frame.dlc < 2) {
    return;
  }

  if (frame.data[0] == static_cast<uint8_t>(CMD_LASER_CONTROL + port_id_)) {
    laser_state_ = frame.data[1] ? 1.0 : 0.0;
    RCLCPP_INFO(
      rclcpp::get_logger("LaserHardwareInterface"),
      "Laser state confirmed from CAN: %s", laser_state_ > 0.5 ? "ON" : "OFF");
  }
}

std::vector<hardware_interface::StateInterface>
LaserHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  const std::string & name = info_.gpios[0].name;

  state_interfaces.emplace_back(
    hardware_interface::StateInterface(name, "laser_state", &laser_state_));

  state_interfaces.emplace_back(
    hardware_interface::StateInterface(name, "is_connected", &is_connected_));

  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Exported %zu state interfaces", state_interfaces.size());

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
LaserHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  const std::string & name = info_.gpios[0].name;

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(name, "laser_command", &laser_command_));

  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Exported %zu command interfaces", command_interfaces.size());

  return command_interfaces;
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Activating laser hardware...");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Deactivating laser hardware...");

  // Safety: Turn laser OFF on deactivation
  if (can_connected_) {
    can_tx_frame_ = CANLib::CanFrame();
    can_tx_frame_.id = can_id_;
    can_tx_frame_.dlc = 2;
    can_tx_frame_.data[0] = static_cast<uint8_t>(CMD_LASER_CONTROL + port_id_);
    can_tx_frame_.data[1] = 0;
    canBus_.send(can_tx_frame_);

    canBus_.close();
    can_connected_ = false;
    is_connected_ = 0.0;
  }

  laser_state_ = 0.0;
  laser_command_ = 0.0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Cleaning up laser hardware...");

  if (can_connected_) {
    // Ensure laser is OFF before cleanup
    can_tx_frame_ = CANLib::CanFrame();
    can_tx_frame_.id = can_id_;
    can_tx_frame_.dlc = 2;
    can_tx_frame_.data[0] = static_cast<uint8_t>(CMD_LASER_CONTROL + port_id_);
    can_tx_frame_.data[1] = 0;
    canBus_.send(can_tx_frame_);
    canBus_.close();
  }

  can_connected_ = false;
  is_connected_ = 0.0;
  laser_state_ = 0.0;
  laser_command_ = 0.0;

  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Laser hardware cleanup complete");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Shutting down laser hardware...");

  return on_cleanup(previous_state);
}

hardware_interface::return_type LaserHardwareInterface::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // CAN messages are handled asynchronously via callback
  // State is updated in onCanMessage()
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LaserHardwareInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  bool commanded_on = (laser_command_ > 0.5);
  bool currently_on = (laser_state_ > 0.5);

  if (commanded_on != currently_on) {
    if (can_connected_) {
      can_tx_frame_ = CANLib::CanFrame();
      can_tx_frame_.id = can_id_;
      can_tx_frame_.dlc = 2;
      can_tx_frame_.data[0] = static_cast<uint8_t>(CMD_LASER_CONTROL + port_id_);
      can_tx_frame_.data[1] = commanded_on ? 1 : 0;
      canBus_.send(can_tx_frame_);

      RCLCPP_INFO(
        rclcpp::get_logger("LaserHardwareInterface"),
        "Laser command sent: %s", commanded_on ? "ON" : "OFF");
    } else {
      // Simulation mode — update state immediately, no hardware to confirm
      laser_state_ = commanded_on ? 1.0 : 0.0;
      RCLCPP_INFO(
        rclcpp::get_logger("LaserHardwareInterface"),
        "Laser turned %s (simulated)", commanded_on ? "ON" : "OFF");
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace laser_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  laser_ros2_control::LaserHardwareInterface,
  hardware_interface::SystemInterface)