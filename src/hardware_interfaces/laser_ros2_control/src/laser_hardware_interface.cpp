#include "laser_ros2_control/laser_hardware_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include <sstream>

namespace laser_ros2_control
{

void LaserHardwareInterface::logger_function()
{
  if (LASERJoints_.empty()) {
    return;
  }

  const auto & joint = LASERJoints_.front();
  std::ostringstream oss;
  oss << "\033[2J\033[H \nLASER Logger"
      << "\n--- HWI Specific ---\n"
      << "CAN Interface: " << can_interface_
      << " | HWI Update Rate: " << update_rate_
      << " | Logger Update Rate: " << logger_rate_ << "\n"
      << "Elapsed Time since first update: " << elapsed_time_ << "\n"
      << "\n--- Joint Specific ---\n"
      << "JOINT: " << joint.name << "\n"
      << "Parameters: CAN ID: 0x" << std::hex << std::uppercase << joint.can_id << std::dec << "\n"
      << "-- Commands --\n"
      << "Laser Command: " << joint.laser_command
      << " | Status Request: " << joint.status_request << "\n"
      << "-- State --\n"
      << "Laser State: " << joint.laser_state
      << " | Temperature: " << joint.temperature
      << " | Is Connected: " << joint.is_connected
      << " | Status: " << joint.status << "\n";

  RCLCPP_INFO(rclcpp::get_logger("LaserHardwareInterface"), "%s", oss.str().c_str());
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_init(
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
    std::stoi(info_.hardware_parameters.at("update_rate")) : 0;
  logger_rate_ = info_.hardware_parameters.count("logger_rate") ?
    std::stoi(info_.hardware_parameters.at("logger_rate")) : 0;
  logger_state_ = info_.hardware_parameters.count("logger_state") ?
    std::stoi(info_.hardware_parameters.at("logger_state")) : 0;

  can_id_ = info_.hardware_parameters.count("can_id") ?
    static_cast<uint32_t>(std::stoul(info_.hardware_parameters.at("can_id"), nullptr, 0)) : 0x130;

  // port_id is the lower nibble of can_id by convention, or default to 0
  port_id_ = info_.hardware_parameters.count("port_id") ?
    static_cast<uint8_t>(std::stoul(info_.hardware_parameters.at("port_id"), nullptr, 0)) : 0x00;

  LASERJoints_.clear();
  LASERJoints_.push_back(LaserJoint{
    info_.gpios[0].name,
    can_id_,
    0.0,  // laser_state
    0.0,  // temperature
    0.0,  // is_connected
    0.0,  // status
    0.0,  // laser_command
    0.0,  // status_request
    0.0,  // prev_status_request
    0.0   // elapsed_status_request_time
  });

  can_connected_       = false;
  elapsed_time_        = 0.0;
  elapsed_logger_time_ = 0.0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Configuring laser hardware...");

  // Close existing connection if re-configuring
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

  auto & joint       = LASERJoints_.front();
  joint.is_connected = can_connected_ ? 1.0 : 0.0;
  joint.status       = joint.is_connected;

  RCLCPP_INFO(
    rclcpp::get_logger("LaserHardwareInterface"),
    "Laser hardware configured (%s)", can_connected_ ? "CAN MODE" : "SIMULATION");

  return hardware_interface::CallbackReturn::SUCCESS;
}

void LaserHardwareInterface::onCanMessage(const CANLib::CanFrame & frame)
{
  auto & joint = LASERJoints_.front();

  if (frame.id != can_id_ || frame.dlc < 2) {
    return;
  }

  // 0x20 + port_id: laser control confirmation (protocol 0x20 response)
  if (frame.data[0] == static_cast<uint8_t>(CMD_LASER_CONTROL + port_id_)) {
    const bool success = frame.data[1] != 0;
    joint.status = success ? 1.0 : 0.0;
    if (success) {
      // Confirm the state we commanded is now reflected in hardware
      joint.laser_state = joint.laser_command > 0.5 ? 1.0 : 0.0;
    }
    RCLCPP_INFO(
      rclcpp::get_logger("LaserHardwareInterface"),
      "Laser control %s from CAN", success ? "confirmed" : "failed");
    return;
  }

  // 0x30 + port_id: status response (protocol 0x30 response)
  if (frame.data[0] == static_cast<uint8_t>(CMD_LASER_STATUS + port_id_)) {
    const bool success = frame.data[1] != 0;
    joint.status = success ? 1.0 : 0.0;
    RCLCPP_INFO(
      rclcpp::get_logger("LaserHardwareInterface"),
      "Laser status response: %s", success ? "OK" : "FAILED");
    return;
  }
}

std::vector<hardware_interface::StateInterface> LaserHardwareInterface::export_state_interfaces()
{
  auto & joint = LASERJoints_.front();
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(joint.name, "laser_state",  &joint.laser_state);
  state_interfaces.emplace_back(joint.name, "temperature",  &joint.temperature);
  state_interfaces.emplace_back(joint.name, "is_connected", &joint.is_connected);
  state_interfaces.emplace_back(joint.name, "status",       &joint.status);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> LaserHardwareInterface::export_command_interfaces()
{
  auto & joint = LASERJoints_.front();
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(joint.name, "laser_command",  &joint.laser_command);
  command_interfaces.emplace_back(joint.name, "status_request", &joint.status_request);
  return command_interfaces;
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  auto & joint = LASERJoints_.front();

  // Safety: send laser OFF before deactivating
  if (can_connected_) {
    can_tx_frame_         = CANLib::CanFrame();
    can_tx_frame_.id      = joint.can_id;
    can_tx_frame_.dlc     = 2;
    can_tx_frame_.data[0] = static_cast<uint8_t>(CMD_LASER_CONTROL + port_id_);
    can_tx_frame_.data[1] = 0;
    canBus_.send(can_tx_frame_);

    canBus_.close();
    can_connected_     = false;
    joint.is_connected = 0.0;
  }

  joint.laser_state   = 0.0;
  joint.laser_command = 0.0;
  joint.status        = 0.0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  auto & joint = LASERJoints_.front();

  if (can_connected_) {
    can_tx_frame_         = CANLib::CanFrame();
    can_tx_frame_.id      = joint.can_id;
    can_tx_frame_.dlc     = 2;
    can_tx_frame_.data[0] = static_cast<uint8_t>(CMD_LASER_CONTROL + port_id_);
    can_tx_frame_.data[1] = 0;
    canBus_.send(can_tx_frame_);
    canBus_.close();
  }

  can_connected_     = false;
  joint.is_connected = 0.0;
  joint.laser_state  = 0.0;
  joint.laser_command = 0.0;
  joint.status        = 0.0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LaserHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

hardware_interface::return_type LaserHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LaserHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  auto & joint = LASERJoints_.front();

  elapsed_time_        += period.seconds();
  elapsed_logger_time_ += period.seconds();
  if (logger_rate_ > 0 &&
      elapsed_logger_time_ > (1.0 / static_cast<double>(logger_rate_)))
  {
    elapsed_logger_time_ = 0.0;
    if (logger_state_ == 1) {
      logger_function();
    }
  }

  // --- Laser on/off: send on state change ---
  const bool commanded_on = joint.laser_command > 0.5;
  const bool currently_on = joint.laser_state   > 0.5;
  if (commanded_on != currently_on) {
    if (can_connected_) {
      can_tx_frame_         = CANLib::CanFrame();
      can_tx_frame_.id      = joint.can_id;
      can_tx_frame_.dlc     = 2;
      can_tx_frame_.data[0] = static_cast<uint8_t>(CMD_LASER_CONTROL + port_id_);
      can_tx_frame_.data[1] = commanded_on ? 1 : 0;
      canBus_.send(can_tx_frame_);
      RCLCPP_INFO(
        rclcpp::get_logger("LaserHardwareInterface"),
        "Laser command sent: %s", commanded_on ? "ON" : "OFF");
    } else {
      // Simulation mode: update state immediately, no hardware to confirm
      joint.laser_state = commanded_on ? 1.0 : 0.0;
      RCLCPP_INFO(
        rclcpp::get_logger("LaserHardwareInterface"),
        "Laser turned %s (simulated)", commanded_on ? "ON" : "OFF");
    }
  }

  // --- Status request: one-shot (negative) or heartbeat (positive Hz) ---
  const double curr_status_req = joint.status_request;
  if (curr_status_req < 0.0 && joint.prev_status_request >= 0.0) {
    if (can_connected_) {
      can_tx_frame_         = CANLib::CanFrame();
      can_tx_frame_.id      = joint.can_id;
      can_tx_frame_.dlc     = 2;
      can_tx_frame_.data[0] = static_cast<uint8_t>(CMD_LASER_STATUS + port_id_);
      can_tx_frame_.data[1] = 1;
      canBus_.send(can_tx_frame_);
    }
  } else if (curr_status_req > 0.0) {
    joint.elapsed_status_request_time += period.seconds();
    if (joint.elapsed_status_request_time > (1.0 / curr_status_req)) {
      joint.elapsed_status_request_time = 0.0;
      if (can_connected_) {
        can_tx_frame_         = CANLib::CanFrame();
        can_tx_frame_.id      = joint.can_id;
        can_tx_frame_.dlc     = 2;
        can_tx_frame_.data[0] = static_cast<uint8_t>(CMD_LASER_STATUS + port_id_);
        can_tx_frame_.data[1] = 1;
        canBus_.send(can_tx_frame_);
      }
    }
  }
  joint.prev_status_request = curr_status_req;

  return hardware_interface::return_type::OK;
}

}  // namespace laser_ros2_control

PLUGINLIB_EXPORT_CLASS(
  laser_ros2_control::LaserHardwareInterface,
  hardware_interface::SystemInterface)