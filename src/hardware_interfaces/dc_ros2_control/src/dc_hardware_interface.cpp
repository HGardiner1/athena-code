#include "dc_ros2_control/dc_hardware_interface.hpp"

#include <netdb.h>
#include <sys/socket.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <cstring>
#include <sstream>
#include <string>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dc_ros2_control
{

// =============================================================================
// Unit Conversions (matches servo HWI structure)
// =============================================================================

double DCHardwareInterface::calculate_joint_position_from_motor_position(double motor_pos_deg, int gear_ratio) {
  // deg -> radians with gear ratio
  return (motor_pos_deg * (M_PI / 180.0)) / gear_ratio;
}

double DCHardwareInterface::calculate_joint_displacement_from_motor_position(double motor_pos_deg, int gear_ratio, double dist_per_rev) {
  // deg -> meters: (deg / 360) * distance_per_rev / gear_ratio
  return (motor_pos_deg / 360.0) * dist_per_rev / gear_ratio;
}

double DCHardwareInterface::calculate_joint_angular_velocity_from_motor_velocity(double motor_vel_dps, int gear_ratio) {
  // deg/s -> rad/s with gear ratio
  return (motor_vel_dps * (M_PI / 180.0)) / gear_ratio;
}

double DCHardwareInterface::calculate_joint_linear_velocity_from_motor_velocity(double motor_vel_dps, int gear_ratio, double dist_per_rev) {
  // deg/s -> m/s
  return (motor_vel_dps / 360.0) * dist_per_rev / gear_ratio;
}

int16_t DCHardwareInterface::calculate_motor_position_from_desired_joint_position(double joint_position, int gear_ratio) {
  // radians -> deg with gear ratio
  return static_cast<int16_t>(std::round((joint_position * (180.0 / M_PI)) * gear_ratio));
}

int16_t DCHardwareInterface::calculate_motor_position_from_desired_joint_displacement(double joint_position, int gear_ratio, double dist_per_rev) {
  // meters -> deg: (m / distance_per_rev) * 360 * gear_ratio
  return static_cast<int16_t>(std::round((joint_position / dist_per_rev) * 360.0 * gear_ratio));
}

int16_t DCHardwareInterface::calculate_motor_velocity_from_desired_joint_angular_velocity(double joint_velocity, int gear_ratio) {
  // rad/s -> deg/s with gear ratio
  return static_cast<int16_t>(std::round((joint_velocity * (180.0 / M_PI)) * gear_ratio));
}

int16_t DCHardwareInterface::calculate_motor_velocity_from_desired_joint_linear_velocity(double joint_velocity, int gear_ratio, double dist_per_rev) {
  // m/s -> deg/s
  return static_cast<int16_t>(std::round((joint_velocity / dist_per_rev) * 360.0 * gear_ratio));
}

// =============================================================================
// Logger
// =============================================================================

void DCHardwareInterface::logger_function()
{
  if (num_joints == 0) return;

  std::string log_msg = "\033[2J\033[H \nDC Motor Logger";
  std::ostringstream oss;
  std::string control_mode;
  std::string joint_type_str;
  std::string status;

  oss << "\n--- HWI Specific ---\n"
      << "CAN Interface: " << can_interface
      << " | Command CAN ID: 0x" << std::hex << std::uppercase << can_command_id
      << " | Response CAN ID: 0x" << std::hex << std::uppercase << can_response_id
      << " | HWI Update Rate: " << std::dec << update_rate
      << " | Logger Update Rate: " << logger_rate << "\n"
      << "Elapsed Time since first update: " << elapsed_time << "\n"
      << "\n--- Joint Specific ---";

  for (int i = 0; i < num_joints; i++) {
    switch (control_level_[i]) {
      case integration_level_t::POSITION:  control_mode = "POSITION"; break;
      case integration_level_t::VELOCITY:  control_mode = "VELOCITY"; break;
      default:                             control_mode = "UNDEFINED"; break;
    }

    switch (joint_type_[i]) {
      case joint_type_t::REVOLUTE:  joint_type_str = "REVOLUTE"; break;
      case joint_type_t::PRISMATIC: joint_type_str = "PRISMATIC"; break;
    }

    switch (device_status[i]) {
      case 0:  status = "Undefined"; break;
      case 1:  status = "Idle"; break;
      case 2:  status = "Startup Sequence"; break;
      case 3:  status = "Error (Invalid Request)"; break;
      case 4:  status = "Error (Motor Disarmed)"; break;
      case 5:  status = "Error (Motor Failed)"; break;
      case 6:  status = "Error (Controller Failed)"; break;
      case 7:  status = "Error (ESTOP Requested)"; break;
      case 8:  status = "Error (Unknown Position)"; break;
      case 9:  status = "Position Control"; break;
      case 10: status = "Velocity Control"; break;
      case 11: status = "Motor Stopped"; break;
      default: status = "UNKNOWN (" + std::to_string(device_status[i]) + ")"; break;
    }

    oss << "\nJOINT: " << info_.joints[i].name << "\n"
        << "Parameters: Node ID: 0x" << std::hex << std::uppercase << joint_node_ids[i]
        << " | Gear Ratio: " << std::dec << joint_gear_ratios[i]
        << " | Inverted: " << (joint_inverted[i] ? "true" : "false")
        << " | Device Status: " << status
        << " | Control Mode: " << control_mode
        << " | Joint Type: " << joint_type_str << "\n"
        << "-- Commands --\n"
        << "Joint Command Position: " << joint_command_position_[i]
        << " | Joint Command Velocity: " << joint_command_velocity_[i] << "\n"
        << "-- State --\n"
        << "Joint Position: " << joint_state_position_[i]
        << " | Joint Velocity: " << joint_state_velocity_[i] << "\n"
        << "-- Telemetry --\n"
        << "Temperature: " << motor_temperature_[i] << " C"
        << " | Torque Current: " << motor_torque_current_[i] << " A\n";
  }

  log_msg += oss.str();
  RCLCPP_INFO(rclcpp::get_logger("DCHardwareInterface"), "%s", log_msg.c_str());
}

// =============================================================================
// Initialization
// =============================================================================

hardware_interface::CallbackReturn DCHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // -- Per-joint parameters --
  for (auto& joint : info_.joints) {
    joint_node_ids.push_back(
      std::clamp(std::stoi(joint.parameters.at("node_id"), nullptr, 0), 0x0, 0xF));

    // Gear ratio (default: 1)
    if (joint.parameters.count("gear_ratio")) {
      joint_gear_ratios.push_back(std::abs(std::stoi(joint.parameters.at("gear_ratio"))));
    } else {
      joint_gear_ratios.push_back(1);
    }

    // Inverted (default: false)
    if (joint.parameters.count("inverted")) {
      joint_inverted.push_back(joint.parameters.at("inverted") == "true");
    } else {
      joint_inverted.push_back(false);
    }

    // Joint type (default: revolute)
    std::string jtype = "revolute";
    if (joint.parameters.count("joint_type")) {
      jtype = joint.parameters.at("joint_type");
    }

    if (jtype == "revolute") {
      joint_type_.push_back(joint_type_t::REVOLUTE);
      max_disp.push_back(std::nan(""));
      distance_per_rev.push_back(0.0);
    } else if (jtype == "prismatic" && joint.parameters.count("max_disp") && joint.parameters.count("distance_per_rev")) {
      joint_type_.push_back(joint_type_t::PRISMATIC);
      max_disp.push_back(std::abs(std::stod(joint.parameters.at("max_disp"))));
      distance_per_rev.push_back(std::stod(joint.parameters.at("distance_per_rev")));
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("DCHardwareInterface"),
        "Invalid joint_type for joint %s. Must be 'revolute' or 'prismatic' (with 'max_disp' and 'distance_per_rev').",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // -- Hardware-level parameters --
  num_joints = static_cast<int>(info_.joints.size());
  update_rate = std::stoi(info_.hardware_parameters.at("update_rate"));
  logger_rate = std::stoi(info_.hardware_parameters.at("logger_rate"));
  logger_state = std::stoi(info_.hardware_parameters.at("logger_state"));
  can_interface = info_.hardware_parameters.at("can_interface");
  can_command_id = std::stoi(info_.hardware_parameters.at("can_id"), nullptr, 0);
  can_response_id = can_command_id + 0x01;

  elapsed_update_time = 0.0;
  elapsed_time = 0.0;
  elapsed_logger_time = 0.0;
  write_count = 0;

  // -- Command and State Interface initialization --
  joint_state_position_.assign(num_joints, std::numeric_limits<double>::quiet_NaN());
  prev_joint_state_position_.assign(num_joints, std::numeric_limits<double>::quiet_NaN());
  joint_state_velocity_.assign(num_joints, 0.0);
  prev_joint_state_velocity_.assign(num_joints, 0.0);

  joint_command_position_.assign(num_joints, std::numeric_limits<double>::quiet_NaN());
  prev_joint_command_position_.assign(num_joints, std::numeric_limits<double>::quiet_NaN());
  joint_command_velocity_.assign(num_joints, 0.0);
  prev_joint_command_velocity_.assign(num_joints, 0.0);

  motor_position.assign(num_joints, 0.0);
  motor_velocity.assign(num_joints, 0.0);
  device_status.assign(num_joints, 0);

  // Telemetry placeholders (TODO: populate via CAN telemetry command)
  motor_temperature_.assign(num_joints, 0.0);
  motor_torque_current_.assign(num_joints, 0.0);

  // Default control mode: position
  control_level_.resize(num_joints, integration_level_t::POSITION);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// =============================================================================
// Lifecycle
// =============================================================================

hardware_interface::CallbackReturn DCHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return this->on_cleanup(previous_state);
}

std::vector<hardware_interface::StateInterface>
DCHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (int i = 0; i < num_joints; i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_state_position_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_state_velocity_[i]));

    // Telemetry interfaces (TODO: populate via CAN telemetry command)
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "motor_temperature", &motor_temperature_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "torque_current", &motor_torque_current_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DCHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (int i = 0; i < num_joints; i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_command_position_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_command_velocity_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn DCHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DCHardwareInterface"), "Configuring DC Motor HWI...");

  if (!canBus.open(can_interface,
        std::bind(&DCHardwareInterface::on_can_message, this, std::placeholders::_1)))
  {
    RCLCPP_ERROR(rclcpp::get_logger("DCHardwareInterface"),
      "Failed to open CAN interface: %s", can_interface.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("DCHardwareInterface"), "CAN interface opened successfully.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DCHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DCHardwareInterface"), "Cleaning up... please wait...");

  // Send shutdown command (maintenance sub-command 3) to all joints
  for (int i = 0; i < num_joints; i++) {
    can_tx_frame_ = CANLib::CanFrame();
    can_tx_frame_.id = can_command_id;
    can_tx_frame_.dlc = 2;
    can_tx_frame_.data[0] = MAINTENANCE_CMD + (joint_node_ids[i] & 0x0F);
    can_tx_frame_.data[1] = MAINTENANCE_SHUTDOWN;
    canBus.send(can_tx_frame_);
  }

  canBus.close();
  RCLCPP_INFO(rclcpp::get_logger("DCHardwareInterface"), "Cleanup complete.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DCHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DCHardwareInterface"), "Activating... please wait...");

  // Initialize command positions to current state
  joint_command_position_ = joint_state_position_;

  for (size_t i = 0; i < joint_command_position_.size(); ++i) {
    RCLCPP_INFO(rclcpp::get_logger("DCHardwareInterface"),
      "Joint %zu command position initialized to: %f", i, joint_command_position_[i]);
  }

  RCLCPP_INFO(rclcpp::get_logger("DCHardwareInterface"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DCHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DCHardwareInterface"), "Deactivating... please wait...");

  // Send stop command (maintenance sub-command 2) to all joints
  for (int i = 0; i < num_joints; i++) {
    can_tx_frame_ = CANLib::CanFrame();
    can_tx_frame_.id = can_command_id;
    can_tx_frame_.dlc = 2;
    can_tx_frame_.data[0] = MAINTENANCE_CMD + (joint_node_ids[i] & 0x0F);
    can_tx_frame_.data[1] = MAINTENANCE_STOP;
    canBus.send(can_tx_frame_);
  }

  RCLCPP_INFO(rclcpp::get_logger("DCHardwareInterface"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// =============================================================================
// CAN Receive Callback
// =============================================================================

void DCHardwareInterface::on_can_message(const CANLib::CanFrame& frame)
{
  can_rx_frame_ = frame;

  int data[7] = {0x00};
  uint8_t cmd_byte = can_rx_frame_.data[0];
  uint8_t command_nibble = cmd_byte & 0xF0;
  uint8_t device_id_nibble = cmd_byte & 0x0F;
  double raw_motor_position = 0.0;
  double raw_motor_velocity = 0.0;

  for (int i = 0; i < num_joints; i++) {
    if (can_rx_frame_.id != can_response_id || device_id_nibble != joint_node_ids[i])
      continue;

    if (command_nibble == MOTOR_STATE_CMD ||
        command_nibble == ABSOLUTE_POS_CONTROL_CMD ||
        command_nibble == VELOCITY_CONTROL_CMD)
    {
      // DECODING CAN MESSAGE
      data[1] = can_rx_frame_.data[1]; // Position low byte
      data[2] = can_rx_frame_.data[2]; // Position high byte
      data[3] = can_rx_frame_.data[3]; // Velocity low byte
      data[4] = can_rx_frame_.data[4]; // Velocity high byte

      // POSITION: int16 sign extension (NOT int32)
      raw_motor_position = static_cast<double>(static_cast<int16_t>((data[2] << 8) | data[1]));

      // VELOCITY: int16
      raw_motor_velocity = static_cast<double>(static_cast<int16_t>((data[4] << 8) | data[3]));

      // Apply inversion
      double dir = joint_inverted[i] ? -1.0 : 1.0;

      // CALCULATING JOINT STATE
      if (joint_type_[i] == joint_type_t::REVOLUTE) {
        motor_position[i] = dir * calculate_joint_position_from_motor_position(raw_motor_position, joint_gear_ratios[i]);
        motor_velocity[i] = dir * calculate_joint_angular_velocity_from_motor_velocity(raw_motor_velocity, joint_gear_ratios[i]);
      }
      else if (joint_type_[i] == joint_type_t::PRISMATIC) {
        motor_position[i] = dir * calculate_joint_displacement_from_motor_position(raw_motor_position, joint_gear_ratios[i], distance_per_rev[i]);
        motor_velocity[i] = dir * calculate_joint_linear_velocity_from_motor_velocity(raw_motor_velocity, joint_gear_ratios[i], distance_per_rev[i]);
      }
      else {
        RCLCPP_INFO(rclcpp::get_logger("DCHardwareInterface"),
          "The joint type for joint %s is undefined.", info_.joints[i].name.c_str());
      }
    }
    else if (command_nibble == MOTOR_STATUS_CMD)
    {
      device_status[i] = can_rx_frame_.data[1];
    }
    // TODO: Handle telemetry response frames here when implemented
  }
}

// =============================================================================
// Read (update state from CAN data)
// =============================================================================

hardware_interface::return_type DCHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (int i = 0; i < num_joints; i++) {
    if (prev_joint_state_velocity_[i] != motor_velocity[i]) {
      joint_state_velocity_[i] = motor_velocity[i];
      prev_joint_state_velocity_[i] = joint_state_velocity_[i];
    }

    if (prev_joint_state_position_[i] != motor_position[i]) {
      joint_state_position_[i] = motor_position[i];
      prev_joint_state_position_[i] = joint_state_position_[i];
    }

    // Check device status — allow Undefined(0), Idle(1), Position Control(9), Velocity Control(10)
    int s = device_status[i];
    if (s != 0 && s != 1 && s != 9 && s != 10) {
      RCLCPP_ERROR(rclcpp::get_logger("DCHardwareInterface"),
        "Joint %s in error state: %d", info_.joints[i].name.c_str(), s);
      return hardware_interface::return_type::ERROR;
    }

    // TODO: Telemetry — populate motor_temperature_[i] and motor_torque_current_[i] via CAN
  }

  return hardware_interface::return_type::OK;
}

// =============================================================================
// Write (send CAN commands)
// =============================================================================

hardware_interface::return_type DCHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  elapsed_update_time += period.seconds();
  elapsed_time += period.seconds();
  double update_period = 1.0 / update_rate;

  // Logger update
  elapsed_logger_time += period.seconds();
  double logging_period = 1.0 / logger_rate;
  if (elapsed_logger_time > logging_period) {
    elapsed_logger_time = 0.0;
    if (logger_state == 1) {
      logger_function();
    }
  }

  // Rate-limited CAN writes — all joints per tick
  if (elapsed_update_time > update_period) {
    elapsed_update_time = 0.0;

    for (int i = 0; i < num_joints; i++) {
      can_tx_frame_ = CANLib::CanFrame();
      can_tx_frame_.id = can_command_id;

      // Apply inversion for outgoing commands
      double dir = joint_inverted[i] ? -1.0 : 1.0;

      if (control_level_[i] == integration_level_t::POSITION &&
          std::isfinite(joint_command_position_[i]) &&
          joint_command_position_[i] != prev_joint_command_position_[i])
      {
        // Clamp prismatic joints to [0, max_disp]
        if (joint_type_[i] == joint_type_t::PRISMATIC) {
          joint_command_position_[i] = std::clamp(
            joint_command_position_[i], 0.0, max_disp[i]);
        }

        int16_t motor_pos;
        if (joint_type_[i] == joint_type_t::REVOLUTE) {
          motor_pos = calculate_motor_position_from_desired_joint_position(
            dir * joint_command_position_[i], joint_gear_ratios[i]);
        } else {
          motor_pos = calculate_motor_position_from_desired_joint_displacement(
            dir * joint_command_position_[i], joint_gear_ratios[i], distance_per_rev[i]);
        }

        can_tx_frame_.dlc = 3;
        can_tx_frame_.data[0] = ABSOLUTE_POS_CONTROL_CMD + (joint_node_ids[i] & 0x0F);
        can_tx_frame_.data[1] = static_cast<uint8_t>(motor_pos & 0xFF);
        can_tx_frame_.data[2] = static_cast<uint8_t>((motor_pos >> 8) & 0xFF);

        prev_joint_command_position_[i] = joint_command_position_[i];
      }
      else if (control_level_[i] == integration_level_t::VELOCITY &&
               std::isfinite(joint_command_velocity_[i]) &&
               joint_command_velocity_[i] != prev_joint_command_velocity_[i])
      {
        int16_t motor_vel;
        if (joint_type_[i] == joint_type_t::REVOLUTE) {
          motor_vel = calculate_motor_velocity_from_desired_joint_angular_velocity(
            dir * joint_command_velocity_[i], joint_gear_ratios[i]);
        } else {
          motor_vel = calculate_motor_velocity_from_desired_joint_linear_velocity(
            dir * joint_command_velocity_[i], joint_gear_ratios[i], distance_per_rev[i]);
        }

        can_tx_frame_.dlc = 3;
        can_tx_frame_.data[0] = VELOCITY_CONTROL_CMD + (joint_node_ids[i] & 0x0F);
        can_tx_frame_.data[1] = static_cast<uint8_t>(motor_vel & 0xFF);
        can_tx_frame_.data[2] = static_cast<uint8_t>((motor_vel >> 8) & 0xFF);

        prev_joint_command_velocity_[i] = joint_command_velocity_[i];
      }
      else
      {
        // No new command — poll motor state
        can_tx_frame_.dlc = 2;
        can_tx_frame_.data[0] = MOTOR_STATE_CMD + (joint_node_ids[i] & 0x0F);
        can_tx_frame_.data[1] = 1; // Validate request
      }

      canBus.send(can_tx_frame_);
    }
  }

  return hardware_interface::return_type::OK;
}

// =============================================================================
// Command Mode Switch
// =============================================================================

hardware_interface::return_type DCHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string>& start_interfaces,
  const std::vector<std::string>& stop_interfaces)
{
  std::ostringstream ss;
  ss << "perform_command_mode_switch called. start: [";
  for (auto &s : start_interfaces) ss << s << ",";
  ss << "] stop: [";
  for (auto &s : stop_interfaces) ss << s << ",";
  ss << "]";
  RCLCPP_INFO(rclcpp::get_logger("DCHardwareInterface"), "%s", ss.str().c_str());

  std::vector<integration_level_t> requested_modes(num_joints, integration_level_t::UNDEFINED);

  // Process stop interfaces
  for (const auto &ifname : stop_interfaces) {
    for (int i = 0; i < num_joints; ++i) {
      if (ifname.find(info_.joints[i].name) != std::string::npos) {
        requested_modes[i] = integration_level_t::UNDEFINED;
      }
    }
  }

  // Process start interfaces
  for (const auto &ifname : start_interfaces) {
    for (int i = 0; i < num_joints; ++i) {
      const std::string pos_if = info_.joints[i].name + "/" + std::string(hardware_interface::HW_IF_POSITION);
      const std::string vel_if = info_.joints[i].name + "/" + std::string(hardware_interface::HW_IF_VELOCITY);
      if (ifname == pos_if) {
        requested_modes[i] = integration_level_t::POSITION;
      } else if (ifname == vel_if) {
        requested_modes[i] = integration_level_t::VELOCITY;
      }
    }
  }

  // Apply modes
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
        RCLCPP_INFO(rclcpp::get_logger("DCHardwareInterface"),
          "Joint %s: stopped -> UNDEFINED", info_.joints[i].name.c_str());
      }
    } else {
      control_level_[i] = requested_modes[i];
      if (requested_modes[i] == integration_level_t::VELOCITY) {
        joint_command_velocity_[i] = 0;
        RCLCPP_INFO(rclcpp::get_logger("DCHardwareInterface"),
          "Joint %s: switched to VELOCITY", info_.joints[i].name.c_str());
      } else if (requested_modes[i] == integration_level_t::POSITION) {
        joint_command_position_[i] = joint_state_position_[i];
        RCLCPP_INFO(rclcpp::get_logger("DCHardwareInterface"),
          "Joint %s: switched to POSITION (initialized to %f)",
          info_.joints[i].name.c_str(), joint_command_position_[i]);
      }
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace dc_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dc_ros2_control::DCHardwareInterface, hardware_interface::SystemInterface)