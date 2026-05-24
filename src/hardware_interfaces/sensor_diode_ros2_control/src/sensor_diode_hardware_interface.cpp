#include "sensor_diode_ros2_control/sensor_diode_hardware_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include <sstream>

namespace sensor_diode_ros2_control
{

void SensorDiodeHardwareInterface::logger_function()
{
  if (DIODEJoints_.empty()) return;
  const auto & joint = DIODEJoints_.front();

  std::ostringstream oss;
  oss << "\033[2J\033[H \nSENSOR DIODE Logger"
      << "\n--- HWI Specific ---\n"
      << "CAN Interface: " << can_interface_
      << " | CAN ID: 0x" << std::hex << std::uppercase << joint.can_id << std::dec
      << " | Port ID: " << static_cast<int>(port_id_)
      << " | Update Rate: " << update_rate_
      << " | Logger Rate: " << logger_rate_ << "\n"
      << "Elapsed Time: " << elapsed_time_ << "\n"
      << "\n--- Joint Specific ---\n"
      << "JOINT: " << joint.name << "\n"
      << "-- Commands --\n"
      << "Request Measurement: " << joint.request_measurement_cmd
      << " | Status Request: "   << joint.status_request
      << " | Maintenance Request: " << joint.maintenance_request << "\n"
      << "-- State --\n"
      << "Wavelength Intensity: " << joint.wavelength_intensity
      << " | Command Success: "   << joint.command_success
      << " | Is Connected: "      << joint.is_connected
      << " | Status: "            << joint.status << "\n";

  RCLCPP_INFO(rclcpp::get_logger("SensorDiodeHardwareInterface"), "%s", oss.str().c_str());
}

hardware_interface::CallbackReturn SensorDiodeHardwareInterface::on_init(
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

  can_id_ = info_.hardware_parameters.count("can_id") ?
    static_cast<uint32_t>(
      std::stoul(info_.hardware_parameters.at("can_id"), nullptr, 0)) : 0x110;
  port_id_ = info_.hardware_parameters.count("port_id") ?
    static_cast<uint8_t>(
      std::stoul(info_.hardware_parameters.at("port_id"), nullptr, 0)) : 0x00;

  DIODEJoints_.clear();
  DIODEJoints_.push_back(SensorDiodeJoint{
    info_.gpios[0].name,
    can_id_,
    // state
    0.0, 0.0, 0.0, 0.0,
    // command
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    // tracking
    0.0, 0.0, 0.0, 0.0, 0.0,
    // internal
    false, 0.0
  });

  can_connected_       = false;
  elapsed_time_        = 0.0;
  elapsed_logger_time_ = 0.0;

  RCLCPP_INFO(
    rclcpp::get_logger("SensorDiodeHardwareInterface"),
    "Initialized sensor diode on %s, CAN ID 0x%X, port %u",
    can_interface_.c_str(), can_id_, port_id_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorDiodeHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
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
      "Successfully opened CAN interface %s", can_interface_.c_str());
  }

  auto & joint           = DIODEJoints_.front();
  joint.is_connected     = can_connected_ ? 1.0 : 0.0;
  joint.status           = joint.is_connected;
  joint.command_success  = 0.0;
  joint.awaiting_response = false;

  RCLCPP_INFO(
    rclcpp::get_logger("SensorDiodeHardwareInterface"),
    "Sensor diode configured (%s)", can_connected_ ? "CAN MODE" : "SIMULATION");

  return hardware_interface::CallbackReturn::SUCCESS;
}

void SensorDiodeHardwareInterface::onCanMessage(const CANLib::CanFrame & frame)
{
  if (frame.id != can_id_ || frame.dlc < 2) return;

  auto & joint = DIODEJoints_.front();

  // 0x20 + port_id: diode value response
  if (frame.data[0] == static_cast<uint8_t>(CMD_READ_DIODE_VALUE + port_id_)) {
    joint.wavelength_intensity = static_cast<double>(frame.data[1]);
    joint.command_success      = 1.0;
    joint.status               = 1.0;
    joint.awaiting_response    = false;
    RCLCPP_DEBUG(
      rclcpp::get_logger("SensorDiodeHardwareInterface"),
      "Diode value received: %.0f", joint.wavelength_intensity);
  }
}

std::vector<hardware_interface::StateInterface>
SensorDiodeHardwareInterface::export_state_interfaces()
{
  auto & joint = DIODEJoints_.front();
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(joint.name, "wavelength_intensity", &joint.wavelength_intensity);
  state_interfaces.emplace_back(joint.name, "command_success",      &joint.command_success);
  state_interfaces.emplace_back(joint.name, "is_connected",         &joint.is_connected);
  state_interfaces.emplace_back(joint.name, "status",               &joint.status);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SensorDiodeHardwareInterface::export_command_interfaces()
{
  auto & joint = DIODEJoints_.front();
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(joint.name, "request_measurement",    &joint.request_measurement_cmd);
  command_interfaces.emplace_back(joint.name, "status_request",         &joint.status_request);
  command_interfaces.emplace_back(joint.name, "maintenance_request",    &joint.maintenance_request);
  command_interfaces.emplace_back(joint.name, "maintenance_frame_high", &joint.maintenance_frame_high);
  command_interfaces.emplace_back(joint.name, "maintenance_frame_low",  &joint.maintenance_frame_low);
  command_interfaces.emplace_back(joint.name, "maintenance_data_count", &joint.maintenance_data_count);
  return command_interfaces;
}

hardware_interface::CallbackReturn SensorDiodeHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorDiodeHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  auto & joint = DIODEJoints_.front();
  joint.awaiting_response        = false;
  joint.request_measurement_cmd = 0.0;

  if (can_connected_) {
    canBus_.close();
    can_connected_     = false;
    joint.is_connected = 0.0;
    joint.status       = 0.0;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorDiodeHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  auto & joint = DIODEJoints_.front();

  if (can_connected_) {
    canBus_.close();
  }

  can_connected_              = false;
  joint.is_connected          = 0.0;
  joint.status                = 0.0;
  joint.wavelength_intensity  = 0.0;
  joint.command_success       = 0.0;
  joint.awaiting_response     = false;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorDiodeHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

hardware_interface::return_type SensorDiodeHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SensorDiodeHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  auto & joint = DIODEJoints_.front();

  elapsed_time_        += period.seconds();
  elapsed_logger_time_ += period.seconds();
  if (logger_rate_ > 0 &&
      elapsed_logger_time_ > (1.0 / static_cast<double>(logger_rate_)))
  {
    elapsed_logger_time_ = 0.0;
    if (logger_state_ == 1) logger_function();
  }

  // --- Measurement request: one-shot on rising edge > 0.5 ---
  if (joint.request_measurement_cmd > 0.5 && joint.prev_request_measurement_cmd <= 0.5) {
    joint.command_success      = 0.0;
    joint.awaiting_response    = true;
    if (can_connected_) {
      can_tx_frame_         = CANLib::CanFrame();
      can_tx_frame_.id      = joint.can_id;
      can_tx_frame_.dlc     = 2;
      can_tx_frame_.data[0] = static_cast<uint8_t>(CMD_READ_DIODE_VALUE + port_id_);
      can_tx_frame_.data[1] = CONFIRM_SEND;
      canBus_.send(can_tx_frame_);
      RCLCPP_INFO(
        rclcpp::get_logger("SensorDiodeHardwareInterface"),
        "Requested diode measurement on port %u", port_id_);
    }
  }
  joint.prev_request_measurement_cmd = joint.request_measurement_cmd;

  // --- Status request: one-shot (negative) or heartbeat (positive Hz) ---
  const double curr_status_req = joint.status_request;
  if (curr_status_req < 0.0 && joint.prev_status_request >= 0.0) {
    if (can_connected_) {
      can_tx_frame_         = CANLib::CanFrame();
      can_tx_frame_.id      = joint.can_id;
      can_tx_frame_.dlc     = 2;
      can_tx_frame_.data[0] = static_cast<uint8_t>(CMD_READ_DIODE_VALUE + port_id_);
      can_tx_frame_.data[1] = CONFIRM_SEND;
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
        can_tx_frame_.data[0] = static_cast<uint8_t>(CMD_READ_DIODE_VALUE + port_id_);
        can_tx_frame_.data[1] = CONFIRM_SEND;
        canBus_.send(can_tx_frame_);
      }
    }
  }
  joint.prev_status_request = curr_status_req;

  // --- Maintenance request: one-shot (negative) or heartbeat (positive Hz) ---
  auto doubles_to_payload = [](double high, double low) -> int64_t {
    return static_cast<int64_t>(
      (static_cast<uint64_t>(high) << 32) | static_cast<uint64_t>(low));
  };

  joint.maintenance_frame = static_cast<double>(doubles_to_payload(
    joint.maintenance_frame_high, joint.maintenance_frame_low));
  const auto decoded = unpack_command_full(
    static_cast<int32_t>(joint.maintenance_data_count),
    static_cast<int64_t>(joint.maintenance_frame));

  // Only send if the decoded command has valid u8 payload
  if (decoded.u8_data.size() == 1 &&
      decoded.i16_data.empty() &&
      decoded.i32_data.empty())
  {
    CANLib::CanFrame maint_frame{};
    maint_frame.id      = joint.can_id;
    maint_frame.dlc     = 2;
    maint_frame.data[0] = decoded.command_id;
    maint_frame.data[1] = decoded.u8_data[0];

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

}  // namespace sensor_diode_ros2_control

PLUGINLIB_EXPORT_CLASS(
  sensor_diode_ros2_control::SensorDiodeHardwareInterface,
  hardware_interface::SystemInterface)