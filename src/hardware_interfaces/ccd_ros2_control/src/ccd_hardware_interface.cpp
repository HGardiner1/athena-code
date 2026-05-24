#include "ccd_ros2_control/ccd_hardware_interface.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <sstream>

namespace ccd_ros2_control
{

void CCDHardwareInterface::logger_function()
{
  if (CCDJoints_.empty()) return;
  const auto & joint = CCDJoints_.front();

  std::ostringstream oss;
  oss << "\033[2J\033[H \nCCD Logger"
      << "\n--- HWI Specific ---\n"
      << "CAN Interface: " << can_interface_
      << " | CAN ID: 0x" << std::hex << std::uppercase << joint.can_id << std::dec
      << " | Update Rate: " << update_rate_
      << " | Logger Rate: " << logger_rate_ << "\n"
      << "Elapsed Time: " << elapsed_time_ << "\n"
      << "\n--- Joint Specific ---\n"
      << "JOINT: " << joint.name << "\n"
      << "-- Commands --\n"
      << "Capture Binary: " << joint.capture_binary_cmd
      << " | Capture Byte: " << joint.capture_byte_cmd << "\n"
      << "Status Request: " << joint.status_request
      << " | Maintenance Request: " << joint.maintenance_request << "\n"
      << "-- State --\n"
      << "Is Connected: "           << joint.is_connected
      << " | Command Success: "     << joint.command_success
      << " | Acquisition Active: "  << joint.acquisition_in_progress
      << " | Data Ready: "          << joint.data_ready << "\n"
      << "Frames Received: "        << joint.frames_received
      << " | Last Frame ID: "       << joint.last_frame_id
      << " | Status: "              << joint.status << "\n";

  RCLCPP_INFO(rclcpp::get_logger("CCDHardwareInterface"), "%s", oss.str().c_str());
}

bool CCDHardwareInterface::format_maintenance_command(
  CANLib::CanFrame & frame, uint32_t can_id, const DecodedCommand & decoded_cmd)
{
  frame.data.fill(0);
  frame.id  = can_id;
  frame.dlc = 2;
  frame.data[0] = decoded_cmd.command_id;
  if (decoded_cmd.u8_data.size() != 1 ||
      !decoded_cmd.i16_data.empty() ||
      !decoded_cmd.i32_data.empty())
  {
    return false;
  }
  frame.data[1] = decoded_cmd.u8_data[0];
  return true;
}

hardware_interface::CallbackReturn CCDHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  can_interface_ = info_.hardware_parameters.count("can_interface") ?
    info_.hardware_parameters.at("can_interface") : "can0";
  update_rate_ = info_.hardware_parameters.count("update_rate") ?
    std::stoi(info_.hardware_parameters.at("update_rate")) : 10;
  logger_rate_ = info_.hardware_parameters.count("logger_rate") ?
    std::stoi(info_.hardware_parameters.at("logger_rate")) : 5;
  logger_state_ = info_.hardware_parameters.count("logger_state") ?
    std::stoi(info_.hardware_parameters.at("logger_state")) : 0;

  uint32_t can_id = 0x100;
  if (info_.hardware_parameters.count("can_id")) {
    try {
      can_id = static_cast<uint32_t>(
        std::stoul(info_.hardware_parameters.at("can_id"), nullptr, 0));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("CCDHardwareInterface"),
        "Failed to parse 'can_id': %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  CCDJoints_.clear();
  CCDJoints_.push_back(CCDJoint{
    info_.gpios[0].name,
    can_id,
    // state
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    // command
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    // tracking
    0.0, 0.0, 0.0, 0.0,
    // internal
    false, false, 0.0,
    {},
    std::vector<uint8_t>(3648, 0),
    std::vector<uint8_t>(3648, 0)
  });

  can_connected_       = false;
  elapsed_time_        = 0.0;
  elapsed_logger_time_ = 0.0;

  RCLCPP_INFO(rclcpp::get_logger("CCDHardwareInterface"),
    "CCD initialized on %s, CAN ID 0x%X", can_interface_.c_str(), can_id);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CCDHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto & joint = CCDJoints_.front();

  if (can_connected_) {
    canBus_.close();
    can_connected_ = false;
  }

  if (!canBus_.open(can_interface_,
      std::bind(&CCDHardwareInterface::on_can_message, this, std::placeholders::_1)))
  {
    RCLCPP_WARN(rclcpp::get_logger("CCDHardwareInterface"),
      "Failed to open CAN interface %s", can_interface_.c_str());
    can_connected_ = false;
  } else {
    can_connected_ = true;
  }

  joint.is_connected           = can_connected_ ? 1.0 : 0.0;
  joint.status                 = joint.is_connected;
  joint.command_success        = 0.0;
  joint.acquisition_in_progress = 0.0;
  joint.data_ready             = 0.0;
  joint.frames_received        = 0.0;
  joint.last_frame_id          = 0.0;

  pixel_pub_node_ = rclcpp::Node::make_shared("ccd_hwi_pixel_publisher");
  pixel_publisher_ = pixel_pub_node_->create_publisher<msgs::msg::RamanSpectrum>(
    "/raman/raw_pixels", rclcpp::QoS(1).reliable());

  return hardware_interface::CallbackReturn::SUCCESS;
}

void CCDHardwareInterface::on_can_message(const CANLib::CanFrame & frame)
{
  auto & joint = CCDJoints_.front();

  // ACK frame on 0x101 — success/fail for the last request
  if (frame.id == CAN_RESP_ACK_ID && frame.dlc >= 2) {
    const uint8_t cmd_byte = frame.data[0];
    const bool success     = frame.data[1] != 0;
    joint.command_success  = success ? 1.0 : 0.0;
    joint.status           = success ? 1.0 : 0.0;

    if (cmd_byte == CMD_REQUEST_BINARY) {
      if (success) {
        joint.waiting_for_binary_data  = true;
        joint.waiting_for_byte_data    = false;
        joint.acquisition_in_progress  = 1.0;
        joint.data_ready               = 0.0;
        joint.frames_received          = 0.0;
        joint.last_frame_id            = 0.0;
        std::fill(joint.binary_pixels.begin(), joint.binary_pixels.end(), 0);
      } else {
        joint.acquisition_in_progress = 0.0;
      }
    } else if (cmd_byte == CMD_REQUEST_BYTE) {
      if (success) {
        joint.waiting_for_byte_data    = true;
        joint.waiting_for_binary_data  = false;
        joint.acquisition_in_progress  = 1.0;
        joint.data_ready               = 0.0;
        joint.frames_received          = 0.0;
        joint.last_frame_id            = 0.0;
        std::fill(joint.byte_pixels.begin(), joint.byte_pixels.end(), 0);
      } else {
        joint.acquisition_in_progress = 0.0;
      }
    }
    return;
  }

  // Binary data frames on 0x102
  if (frame.id == CAN_RESP_BINARY_ID && joint.waiting_for_binary_data && frame.dlc == 8) {
    const uint16_t frame_id = static_cast<uint16_t>(frame.data[0]);
    joint.last_frame_id  = static_cast<double>(frame_id);
    joint.frames_received += 1.0;

    const size_t start_idx = static_cast<size_t>(frame_id) * 56;
    for (size_t byte_idx = 0; byte_idx < 7; ++byte_idx) {
      const uint8_t packed = frame.data[1 + byte_idx];
      for (size_t bit = 0; bit < 8; ++bit) {
        const size_t pixel_idx = start_idx + byte_idx * 8 + bit;
        if (pixel_idx < joint.binary_pixels.size()) {
          joint.binary_pixels[pixel_idx] = static_cast<uint8_t>((packed >> bit) & 0x01);
        }
      }
    }

    if (joint.frames_received >= 66.0) {
      joint.waiting_for_binary_data  = false;
      joint.acquisition_in_progress  = 0.0;
      joint.data_ready               = 1.0;
      RCLCPP_INFO(rclcpp::get_logger("CCDHardwareInterface"), "Binary acquisition complete");
    }
    return;
  }

  // Byte data frames on 0x103
  if (frame.id == CAN_RESP_BYTE_ID && joint.waiting_for_byte_data && frame.dlc == 8) {
    const uint16_t frame_id =
      static_cast<uint16_t>(frame.data[0]) |
      (static_cast<uint16_t>(frame.data[1]) << 8);

    joint.last_frame_id   = static_cast<double>(frame_id);
    joint.frames_received += 1.0;

    const size_t start_idx = static_cast<size_t>(frame_id) * 6;
    for (size_t i = 0; i < 6; ++i) {
      const size_t pixel_idx = start_idx + i;
      if (pixel_idx < joint.byte_pixels.size()) {
        joint.byte_pixels[pixel_idx] = frame.data[2 + i];
      }
    }

    if (joint.frames_received >= 608.0) {
      joint.waiting_for_byte_data   = false;
      joint.acquisition_in_progress = 0.0;
      joint.data_ready              = 1.0;

      // Publish completed pixel buffer
      if (pixel_publisher_) {
        msgs::msg::RamanSpectrum msg;
        msg.header.stamp    = rclcpp::Clock().now();
        msg.header.frame_id = "ccd_sensor";
        msg.spectrometer_id = "pda_spectrometer";
        msg.accumulations   = 1;
        msg.intensities.reserve(joint.byte_pixels.size());
        for (const auto & px : joint.byte_pixels) {
          msg.intensities.push_back(static_cast<double>(px));
        }
        pixel_publisher_->publish(msg);
        rclcpp::spin_some(pixel_pub_node_);
      }
      RCLCPP_INFO(rclcpp::get_logger("CCDHardwareInterface"), "8-bit acquisition complete");
    }
    return;
  }

}

std::vector<hardware_interface::StateInterface>
CCDHardwareInterface::export_state_interfaces()
{
  auto & joint = CCDJoints_.front();
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(joint.name, "is_connected",            &joint.is_connected);
  state_interfaces.emplace_back(joint.name, "command_success",         &joint.command_success);
  state_interfaces.emplace_back(joint.name, "acquisition_in_progress", &joint.acquisition_in_progress);
  state_interfaces.emplace_back(joint.name, "data_ready",              &joint.data_ready);
  state_interfaces.emplace_back(joint.name, "frames_received",         &joint.frames_received);
  state_interfaces.emplace_back(joint.name, "last_frame_id",           &joint.last_frame_id);
  state_interfaces.emplace_back(joint.name, "status",                  &joint.status);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
CCDHardwareInterface::export_command_interfaces()
{
  auto & joint = CCDJoints_.front();
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(joint.name, "capture_binary",         &joint.capture_binary_cmd);
  command_interfaces.emplace_back(joint.name, "capture_byte",           &joint.capture_byte_cmd);
  command_interfaces.emplace_back(joint.name, "status_request",         &joint.status_request);
  command_interfaces.emplace_back(joint.name, "maintenance_request",    &joint.maintenance_request);
  command_interfaces.emplace_back(joint.name, "maintenance_frame_high", &joint.maintenance_frame_high);
  command_interfaces.emplace_back(joint.name, "maintenance_frame_low",  &joint.maintenance_frame_low);
  command_interfaces.emplace_back(joint.name, "maintenance_data_count", &joint.maintenance_data_count);
  return command_interfaces;
}

hardware_interface::CallbackReturn CCDHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CCDHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  auto & joint = CCDJoints_.front();
  joint.acquisition_in_progress  = 0.0;
  joint.waiting_for_binary_data  = false;
  joint.waiting_for_byte_data    = false;

  if (can_connected_) {
    canBus_.close();
    can_connected_     = false;
    joint.is_connected = 0.0;
    joint.status       = 0.0;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CCDHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  auto & joint = CCDJoints_.front();
  if (can_connected_) {
    canBus_.close();
  }
  can_connected_                = false;
  joint.is_connected            = 0.0;
  joint.status                  = 0.0;
  joint.acquisition_in_progress = 0.0;
  joint.waiting_for_binary_data = false;
  joint.waiting_for_byte_data   = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CCDHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

hardware_interface::return_type CCDHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CCDHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  auto & joint = CCDJoints_.front();

  elapsed_time_        += period.seconds();
  elapsed_logger_time_ += period.seconds();
  if (logger_rate_ > 0 &&
      elapsed_logger_time_ > (1.0 / static_cast<double>(logger_rate_)))
  {
    elapsed_logger_time_ = 0.0;
    if (logger_state_ == 1) logger_function();
  }

  // --- Capture commands (one-shot on rising edge) ---
  if (joint.capture_binary_cmd > 0.5) {
    if (can_connected_) {
      can_tx_frame_ = CANLib::CanFrame();
      can_tx_frame_.id      = joint.can_id;
      can_tx_frame_.dlc     = 2;
      can_tx_frame_.data[0] = CMD_REQUEST_BINARY;
      can_tx_frame_.data[1] = CONFIRM_SEND;
      canBus_.send(can_tx_frame_);
    }
    joint.data_ready          = 0.0;
    joint.command_success     = 0.0;
    joint.capture_binary_cmd  = 0.0;
  }

  if (joint.capture_byte_cmd > 0.5) {
    if (can_connected_) {
      can_tx_frame_ = CANLib::CanFrame();
      can_tx_frame_.id      = joint.can_id;
      can_tx_frame_.dlc     = 2;
      can_tx_frame_.data[0] = CMD_REQUEST_BYTE;
      can_tx_frame_.data[1] = CONFIRM_SEND;
      canBus_.send(can_tx_frame_);
    }
    joint.data_ready       = 0.0;
    joint.command_success  = 0.0;
    joint.capture_byte_cmd = 0.0;
  }

  // --- Status request: one-shot (negative) or heartbeat (positive Hz) ---
  const double curr_status_req = joint.status_request;
  if (curr_status_req < 0.0 && joint.prev_status_request >= 0.0) {
    if (can_connected_) {
      can_tx_frame_ = CANLib::CanFrame();
      can_tx_frame_.id      = joint.can_id;
      can_tx_frame_.dlc     = 2;
      can_tx_frame_.data[0] = CMD_REQUEST_BINARY;
      can_tx_frame_.data[1] = CONFIRM_SEND;
      canBus_.send(can_tx_frame_);
    }
  } else if (curr_status_req > 0.0) {
    joint.elapsed_status_request_time += period.seconds();
    if (joint.elapsed_status_request_time > (1.0 / curr_status_req)) {
      joint.elapsed_status_request_time = 0.0;
      if (can_connected_) {
        can_tx_frame_ = CANLib::CanFrame();
        can_tx_frame_.id      = joint.can_id;
        can_tx_frame_.dlc     = 2;
        can_tx_frame_.data[0] = CMD_REQUEST_BINARY;
        can_tx_frame_.data[1] = CONFIRM_SEND;
        canBus_.send(can_tx_frame_);
      }
    }
  }
  joint.prev_status_request = curr_status_req;

  // --- Maintenance request: same one-shot/heartbeat pattern as servo ---
  auto doubles_to_payload = [](double high, double low) -> int64_t {
    return static_cast<int64_t>(
      (static_cast<uint64_t>(high) << 32) | static_cast<uint64_t>(low));
  };

  joint.maintenance_frame = static_cast<double>(doubles_to_payload(
    joint.maintenance_frame_high, joint.maintenance_frame_low));
  const auto decoded_maintenance_cmd = unpack_command_full(
    static_cast<int32_t>(joint.maintenance_data_count),
    static_cast<int64_t>(joint.maintenance_frame));

  CANLib::CanFrame maint_frame;
  if (format_maintenance_command(maint_frame, joint.can_id, decoded_maintenance_cmd)) {
    const double curr_maint_req = joint.maintenance_request;
    if (curr_maint_req < 0.0 && joint.prev_maintenance_request >= 0.0) {
      if (can_connected_) canBus_.send(maint_frame);
    } else if (curr_maint_req > 0.0) {
      joint.elapsed_maintenance_request_time += period.seconds();
      if (joint.elapsed_maintenance_request_time > (1.0 / curr_maint_req)) {
        joint.elapsed_maintenance_request_time = 0.0;
        if (can_connected_) canBus_.send(maint_frame);
      }
    }
    joint.prev_maintenance_request = joint.maintenance_request;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace ccd_ros2_control

PLUGINLIB_EXPORT_CLASS(
  ccd_ros2_control::CCDHardwareInterface,
  hardware_interface::SystemInterface)