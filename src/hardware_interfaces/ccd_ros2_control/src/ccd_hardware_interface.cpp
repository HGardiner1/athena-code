// Copyright (c) 2026 UMD Loop
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include "ccd_ros2_control/ccd_hardware_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <functional>

namespace ccd_ros2_control
{

hardware_interface::CallbackReturn CCDHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != 
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize state variables
  is_connected_ = 0.0;
  command_success_ = 0.0;
  acquisition_in_progress_ = 0.0;
  data_ready_ = 0.0;
  frames_received_ = 0.0;
  last_frame_id_ = 0.0;

  // Initialize command variables
  capture_binary_cmd_ = 0.0;
  capture_byte_cmd_ = 0.0;

  // Initialize internal acquisition state
  waiting_for_binary_data_ = false;
  waiting_for_byte_data_ = false;
  binary_pixels_.assign(3648, 0);
  byte_pixels_.assign(3648, 0);

  // Parse hardware parameters
  if (info_.hardware_parameters.count("can_interface")) {
    can_interface_ = info_.hardware_parameters.at("can_interface");
  } else {
    can_interface_ = "can0";
  }

  if (info_.hardware_parameters.count("can_id")) {
    can_id_ = static_cast<uint32_t>(std::stoul(info_.hardware_parameters.at("can_id"), nullptr, 0));
  } else {
    can_id_ = 0x100;
  }

  can_connected_ = false;

  RCLCPP_INFO(
    rclcpp::get_logger("CCDHardwareInterface"),
    "Initialized CCD on CAN interface %s with ID 0x%X", 
    can_interface_.c_str(), can_id_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CCDHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("CCDHardwareInterface"),
    "Configuring CCD hardware...");

  if (can_connected_) {
    canBus_.close();
    can_connected_ = false;
  }

  if (!canBus_.open(can_interface_, 
      std::bind(&CCDHardwareInterface::onCanMessage, this, std::placeholders::_1))) 
  {
    RCLCPP_WARN(
      rclcpp::get_logger("CCDHardwareInterface"),
      "Failed to open CAN interface %s - running in SIMULATION mode", 
      can_interface_.c_str());
    can_connected_ = false;
  } else {
    can_connected_ = true;
    RCLCPP_INFO(
      rclcpp::get_logger("CCDHardwareInterface"),
      "Successfully opened CAN interface %s", can_interface_.c_str());
  }

  is_connected_ = can_connected_ ? 1.0 : 0.0;
  command_success_ = 0.0;
  acquisition_in_progress_ = 0.0;
  data_ready_ = 0.0;
  frames_received_ = 0.0;
  last_frame_id_ = 0.0;

  RCLCPP_INFO(
    rclcpp::get_logger("CCDHardwareInterface"),
    "CCD hardware configured (%s)", can_connected_ ? "CAN MODE" : "SIMULATION");

  return hardware_interface::CallbackReturn::SUCCESS;
}

void CCDHardwareInterface::onCanMessage(const CANLib::CanFrame& frame)
{
  if (frame.data[0] == CMD_REQUEST_BINARY && frame.dlc >= 2) {
    command_success_ = static_cast<double>(frame.data[1] ? 1.0 : 0.0);

    if (frame.data[1]) {
      waiting_for_binary_data_ = true;
      waiting_for_byte_data_ = false;
      acquisition_in_progress_ = 1.0;
      data_ready_ = 0.0;
      frames_received_ = 0.0;
      last_frame_id_ = 0.0;
      std::fill(binary_pixels_.begin(), binary_pixels_.end(), 0);
    } else {
      acquisition_in_progress_ = 0.0;
    }
    return;
  }

  if (frame.data[0] == CMD_REQUEST_BYTE && frame.dlc >= 2) {
    command_success_ = static_cast<double>(frame.data[1] ? 1.0 : 0.0);

    if (frame.data[1]) {
      waiting_for_byte_data_ = true;
      waiting_for_binary_data_ = false;
      acquisition_in_progress_ = 1.0;
      data_ready_ = 0.0;
      frames_received_ = 0.0;
      last_frame_id_ = 0.0;
      std::fill(byte_pixels_.begin(), byte_pixels_.end(), 0);
    } else {
      acquisition_in_progress_ = 0.0;
    }
    return;
  }

  if (waiting_for_binary_data_ && frame.dlc == 8) {
    const uint16_t frame_id = static_cast<uint16_t>(frame.data[0]);
    last_frame_id_ = static_cast<double>(frame_id);
    frames_received_ += 1.0;

    const size_t start_idx = static_cast<size_t>(frame_id) * 56;
    for (size_t byte_idx = 0; byte_idx < 7; ++byte_idx) {
      const uint8_t packed = frame.data[1 + byte_idx];
      for (size_t bit = 0; bit < 8; ++bit) {
        const size_t pixel_idx = start_idx + byte_idx * 8 + bit;
        if (pixel_idx < binary_pixels_.size()) {
          binary_pixels_[pixel_idx] = static_cast<uint8_t>((packed >> bit) & 0x01);
        }
      }
    }

    // 3648 / 56 = 65 remainder 8, so 66 data frames total
    if (frames_received_ >= 66.0) {
      waiting_for_binary_data_ = false;
      acquisition_in_progress_ = 0.0;
      data_ready_ = 1.0;
      RCLCPP_INFO(
        rclcpp::get_logger("CCDHardwareInterface"),
        "Binary CCD acquisition complete");
      return;
    }
  }

  if (waiting_for_byte_data_ && frame.dlc == 8) {
    const uint16_t frame_id =
      static_cast<uint16_t>(frame.data[0]) |
      (static_cast<uint16_t>(frame.data[1]) << 8);

    last_frame_id_ = static_cast<double>(frame_id);
    frames_received_ += 1.0;

    const size_t start_idx = static_cast<size_t>(frame_id) * 6;
    for (size_t i = 0; i < 6; ++i) {
      const size_t pixel_idx = start_idx + i;
      if (pixel_idx < byte_pixels_.size()) {
        byte_pixels_[pixel_idx] = frame.data[2 + i];
      }
    }

    // 3648 / 6 = 608 data frames total
    if (frames_received_ >= 608.0) {
      waiting_for_byte_data_ = false;
      acquisition_in_progress_ = 0.0;
      data_ready_ = 1.0;
      RCLCPP_INFO(
        rclcpp::get_logger("CCDHardwareInterface"),
        "8-bit CCD acquisition complete");
    }
  }
}

std::vector<hardware_interface::StateInterface> 
CCDHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  const std::string& name = info_.gpios[0].name;

  state_interfaces.emplace_back(
    hardware_interface::StateInterface(name, "is_connected", &is_connected_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(name, "command_success", &command_success_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(name, "acquisition_in_progress", &acquisition_in_progress_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(name, "data_ready", &data_ready_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(name, "frames_received", &frames_received_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(name, "last_frame_id", &last_frame_id_));

  RCLCPP_INFO(
    rclcpp::get_logger("CCDHardwareInterface"),
    "Exported %zu state interfaces", state_interfaces.size());

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> 
CCDHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  const std::string& name = info_.gpios[0].name;

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(name, "capture_binary", &capture_binary_cmd_));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(name, "capture_byte", &capture_byte_cmd_));

  RCLCPP_INFO(
    rclcpp::get_logger("CCDHardwareInterface"),
    "Exported %zu command interfaces", command_interfaces.size());

  return command_interfaces;
}

hardware_interface::CallbackReturn CCDHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("CCDHardwareInterface"),
    "Activating CCD hardware...");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CCDHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("CCDHardwareInterface"),
    "Deactivating CCD hardware...");

  acquisition_in_progress_ = 0.0;
  waiting_for_binary_data_ = false;
  waiting_for_byte_data_ = false;

  if (can_connected_) {
    canBus_.close();
    can_connected_ = false;
    is_connected_ = 0.0;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CCDHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("CCDHardwareInterface"),
    "Cleaning up CCD hardware...");

  if (can_connected_) {
    canBus_.close();
  }

  can_connected_ = false;
  is_connected_ = 0.0;
  acquisition_in_progress_ = 0.0;
  waiting_for_binary_data_ = false;
  waiting_for_byte_data_ = false;

  RCLCPP_INFO(
    rclcpp::get_logger("CCDHardwareInterface"),
    "CCD hardware cleanup complete");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CCDHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(
    rclcpp::get_logger("CCDHardwareInterface"),
    "Shutting down CCD hardware...");

  return on_cleanup(previous_state);
}

hardware_interface::return_type CCDHardwareInterface::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CCDHardwareInterface::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{

  if (capture_binary_cmd_ > 0.5) {
    if (can_connected_) {
      can_tx_frame_ = CANLib::CanFrame();
      can_tx_frame_.id = can_id_;
      can_tx_frame_.dlc = 2;
      can_tx_frame_.data[0] = CMD_REQUEST_BINARY;
      can_tx_frame_.data[1] = 1;
      canBus_.send(can_tx_frame_);
    }

    data_ready_ = 0.0;
    command_success_ = 0.0;
    capture_binary_cmd_ = 0.0;

    RCLCPP_INFO(
      rclcpp::get_logger("CCDHardwareInterface"),
      "Requested binary CCD acquisition%s",
      can_connected_ ? "" : " (simulated)");
  }

  if (capture_byte_cmd_ > 0.5) {
    if (can_connected_) {
      can_tx_frame_ = CANLib::CanFrame();
      can_tx_frame_.id = can_id_;
      can_tx_frame_.dlc = 2;
      can_tx_frame_.data[0] = CMD_REQUEST_BYTE;
      can_tx_frame_.data[1] = 1;
      canBus_.send(can_tx_frame_);
    }

    data_ready_ = 0.0;
    command_success_ = 0.0;
    capture_byte_cmd_ = 0.0;


    RCLCPP_INFO(
      rclcpp::get_logger("CCDHardwareInterface"),
      "Requested byte CCD acquisition%s",
      can_connected_ ? "" : " (simulated)");
  }

  return hardware_interface::return_type::OK;
}

}  // namespace ccd_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ccd_ros2_control::CCDHardwareInterface,
  hardware_interface::SystemInterface)