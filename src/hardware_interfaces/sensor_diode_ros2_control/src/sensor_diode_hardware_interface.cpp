// Copyright (c) 2024 UMD Loop
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "sensor_diode_ros2_control/sensor_diode_hardware_interface.hpp"

#include <functional>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace sensor_diode_ros2_control
{

hardware_interface::CallbackReturn SensorDiodeHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize states
  wavelength_intensity_ = 0.0;
  command_success_ = 0.0;
  is_connected_ = 0.0;

  // Initialize commands
  request_measurement_cmd_ = 0.0;

  // Internal
  awaiting_response_ = false;
  can_connected_ = false;

  // Parse parameters
  if (info_.hardware_parameters.count("can_interface")) {
    can_interface_ = info_.hardware_parameters.at("can_interface");
  } else {
    can_interface_ = "can0";
  }

  if (info_.hardware_parameters.count("can_id")) {
    can_id_ = static_cast<uint32_t>(
      std::stoul(info_.hardware_parameters.at("can_id"), nullptr, 0));
  } else {
    can_id_ = 0x110;
  }

  if (info_.hardware_parameters.count("port_id")) {
    port_id_ = static_cast<uint8_t>(
      std::stoul(info_.hardware_parameters.at("port_id"), nullptr, 0));
  } else {
    port_id_ = 0;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("SensorDiodeHardwareInterface"),
    "Initialized sensor diode on CAN interface %s with CAN ID 0x%X and port ID %u",
    can_interface_.c_str(), can_id_, port_id_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorDiodeHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("SensorDiodeHardwareInterface"),
    "Configuring sensor diode hardware...");

  if (can_connected_) {
    canBus_.close();
    can_connected_ = false;
  }

  if (!canBus_.open(
        can_interface_,
        std::bind(&SensorDiodeHardwareInterface::onCanMessage, this, std::placeholders::_1)))
  {
    RCLCPP_WARN(
      rclcpp::get_logger("SensorDiodeHardwareInterface"),
      "Failed to open CAN interface %s - running in SIMULATION mode",
      can_interface_.c_str());
    can_connected_ = false;
  } else {
    can_connected_ = true;
    RCLCPP_INFO(
      rclcpp::get_logger("SensorDiodeHardwareInterface"),
      "Successfully opened CAN interface %s",
      can_interface_.c_str());
  }

  is_connected_ = can_connected_ ? 1.0 : 0.0;
  command_success_ = 0.0;
  awaiting_response_ = false;

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
SensorDiodeHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  const std::string & name = info_.gpios[0].name;

  state_interfaces.emplace_back(
    hardware_interface::StateInterface(name, "wavelength_intensity", &wavelength_intensity_));

  state_interfaces.emplace_back(
    hardware_interface::StateInterface(name, "command_success", &command_success_));

  state_interfaces.emplace_back(
    hardware_interface::StateInterface(name, "is_connected", &is_connected_));

  RCLCPP_INFO(
    rclcpp::get_logger("SensorDiodeHardwareInterface"),
    "Exported %zu state interfaces",
    state_interfaces.size());

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SensorDiodeHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  const std::string & name = info_.gpios[0].name;

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      name, "request_measurement", &request_measurement_cmd_));

  RCLCPP_INFO(
    rclcpp::get_logger("SensorDiodeHardwareInterface"),
    "Exported %zu command interfaces",
    command_interfaces.size());

  return command_interfaces;
}

hardware_interface::CallbackReturn SensorDiodeHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("SensorDiodeHardwareInterface"),
    "Activating sensor diode hardware...");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorDiodeHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("SensorDiodeHardwareInterface"),
    "Deactivating sensor diode hardware...");

  awaiting_response_ = false;
  request_measurement_cmd_ = 0.0;

  if (can_connected_) {
    canBus_.close();
    can_connected_ = false;
    is_connected_ = 0.0;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorDiodeHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("SensorDiodeHardwareInterface"),
    "Cleaning up sensor diode hardware...");

  if (can_connected_) {
    canBus_.close();
  }

  can_connected_ = false;
  is_connected_ = 0.0;
  awaiting_response_ = false;
  wavelength_intensity_ = 0.0;
  command_success_ = 0.0;

  RCLCPP_INFO(
    rclcpp::get_logger("SensorDiodeHardwareInterface"),
    "Sensor diode hardware cleanup complete");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorDiodeHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(
    rclcpp::get_logger("SensorDiodeHardwareInterface"),
    "Shutting down sensor diode hardware...");

  return on_cleanup(previous_state);
}

void SensorDiodeHardwareInterface::onCanMessage(const CANLib::CanFrame & frame)
{
  if (frame.id != can_id_ || frame.dlc < 2) {
    return;
  }

  const uint8_t expected_cmd = static_cast<uint8_t>(CMD_READ_DIODE_VALUE + port_id_);

  if (frame.data[0] != expected_cmd) {
    return;
  }

  // Response format:
  // data[0] = 0x20 + port_id
  // data[1] = wavelength_intensity [0 to 255]
  wavelength_intensity_ = static_cast<double>(frame.data[1]);
  command_success_ = 1.0;
  awaiting_response_ = false;

  RCLCPP_DEBUG(
    rclcpp::get_logger("SensorDiodeHardwareInterface"),
    "Wavelength Intensity: %.0f",
    wavelength_intensity_);
}

hardware_interface::return_type SensorDiodeHardwareInterface::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // CAN messages are processed asynchronously in the callback.
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SensorDiodeHardwareInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (request_measurement_cmd_ > 0.5) {
    command_success_ = 0.0;
    awaiting_response_ = true;

    if (can_connected_) {
      can_tx_frame_ = CANLib::CanFrame();
      can_tx_frame_.id = can_id_;
      can_tx_frame_.dlc = 2;
      can_tx_frame_.data[0] = static_cast<uint8_t>(CMD_READ_DIODE_VALUE + port_id_);
      can_tx_frame_.data[1] = 1;  // validate request

      canBus_.send(can_tx_frame_);

      RCLCPP_INFO(
        rclcpp::get_logger("SensorDiodeHardwareInterface"),
        "Requested diode measurement on port %u",
        port_id_);
    } else {
      RCLCPP_INFO(
        rclcpp::get_logger("SensorDiodeHardwareInterface"),
        "Requested diode measurement on port %u (simulated)",
        port_id_);
    }

    // Reset trigger after sending
    request_measurement_cmd_ = 0.0;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace sensor_diode_ros2_control

PLUGINLIB_EXPORT_CLASS(
  sensor_diode_ros2_control::SensorDiodeHardwareInterface,
  hardware_interface::SystemInterface)