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

#ifndef CCD_ROS2_CONTROL__CCD_HARDWARE_INTERFACE_HPP_
#define CCD_ROS2_CONTROL__CCD_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <cstdint>
#include <array>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "umdloop_can_library/SocketCanBus.hpp"
#include "umdloop_can_library/CanFrame.hpp"

namespace ccd_ros2_control
{

/**
 * @brief Hardware interface for CAN-controlled spectrometry ccd via ros2_control
 * 
 * This interface controls a spectrometry ccd through CAN bus.
 * 
 * CAN Protocol (ID: 0x100):
 * - ON:  DATA[0] = 0x60
 * - OFF: DATA[0] = 0x80
 * - Read Temperature: DATA[0] = 0x85
 * - Read Wavelength:  DATA[0] = 0x90
 * 
 * State Interfaces (read by controllers):
 * - ccd_state: 0.0 = OFF, 1.0 = ON
 * - temperature: Ccd temperature (if available)
 * - is_connected: Is CAN connected (0.0 or 1.0)
 * 
 * Command Interfaces (written by controllers):
 * - ccd_command: 0.0 = turn OFF, 1.0 = turn ON
 * 
 * Hardware Parameters (from URDF):
 * - can_interface: CAN interface name (default: "can0")
 * - can_id: CAN ID for ccd commands (default: 0x100)
 */
class CCDHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CCDHardwareInterface)

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

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // CAN message handler
  void onCanMessage(const CANLib::CanFrame& frame);

  // Configuration parameters
  std::string can_interface_;
  uint32_t can_id_;

  // CAN bus
  CANLib::SocketCanBus canBus_;
  CANLib::CanFrame can_tx_frame_;
  bool can_connected_;

  // State variables (hardware -> ros2_control)
  double is_connected_;
  double command_success_;
  double acquisition_in_progress_;
  double data_ready_;
  double frames_received_;
  double last_frame_id_;

  // Command variables (ros2_control -> hardware)
  double capture_binary_cmd_;
  double capture_byte_cmd_;

  bool waiting_for_binary_data_;
  bool waiting_for_byte_data_;
  std::vector<uint8_t> binary_pixels_;
  std::vector<uint8_t> byte_pixels_;

  // CAN command bytes
  static constexpr uint8_t CMD_REQUEST_BINARY = 0x20; // Request Binary (1-bit) Measurements
  static constexpr uint8_t CMD_REQUEST_BYTE = 0x30; // Request 8-bit Measurements
};

}  // namespace ccd_ros2_control

#endif  // CCD_ROS2_CONTROL__CCD_HARDWARE_INTERFACE_HPP_