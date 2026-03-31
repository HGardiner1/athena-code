#ifndef DC_HARDWARE_INTERFACE_HPP_
#define DC_HARDWARE_INTERFACE_HPP_

#include <netinet/in.h>
#include <memory>
#include <string>
#include <vector>
#include <cstdint>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include "umdloop_can_library/SocketCanBus.hpp"
#include "umdloop_can_library/CanFrame.hpp"

namespace dc_ros2_control
{
class DCHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DCHardwareInterface)

  // Lifecycle
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // -- Helper Functions --
  void on_can_message(const CANLib::CanFrame& frame);
  void logger_function();

  // Unit conversions
  double calculate_joint_position_from_motor_position(double motor_pos_deg, int gear_ratio);
  double calculate_joint_displacement_from_motor_position(double motor_pos_deg, int gear_ratio, double dist_per_rev);
  double calculate_joint_angular_velocity_from_motor_velocity(double motor_vel_dps, int gear_ratio);
  double calculate_joint_linear_velocity_from_motor_velocity(double motor_vel_dps, int gear_ratio, double dist_per_rev);

  int16_t calculate_motor_position_from_desired_joint_position(double joint_position, int gear_ratio);
  int16_t calculate_motor_position_from_desired_joint_displacement(double joint_position, int gear_ratio, double dist_per_rev);
  int16_t calculate_motor_velocity_from_desired_joint_angular_velocity(double joint_velocity, int gear_ratio);
  int16_t calculate_motor_velocity_from_desired_joint_linear_velocity(double joint_velocity, int gear_ratio, double dist_per_rev);

private:
  // Hardware Interface Parameters
  int update_rate;
  double elapsed_update_time;
  double elapsed_time;
  double elapsed_logger_time;
  int logger_rate;
  int logger_state;
  int write_count;

  int num_joints;

  // Stores Arbitration IDs
  int can_command_id;
  uint32_t can_response_id;

  // Joint state (read from hardware)
  std::vector<double> joint_state_position_;
  std::vector<double> joint_state_velocity_;

  // Previous joint state (change detection)
  std::vector<double> prev_joint_state_position_;
  std::vector<double> prev_joint_state_velocity_;

  // Joint commands (written to hardware)
  std::vector<double> joint_command_position_;
  std::vector<double> joint_command_velocity_;

  // Previous joint commands (change detection)
  std::vector<double> prev_joint_command_position_;
  std::vector<double> prev_joint_command_velocity_;

  // Raw motor data from CAN bus (populated in on_can_message, consumed in read)
  std::vector<double> motor_position;
  std::vector<double> motor_velocity;
  std::vector<int> device_status;

  // Telemetry data (TODO: implement CAN telemetry commands)
  std::vector<double> motor_temperature_;
  std::vector<double> motor_torque_current_;

  // Maximum displacement for prismatic joints
  std::vector<double> max_disp;

  // Prismatic-specific: distance per revolution (future use)
  std::vector<double> distance_per_rev;

  // CAN Library
  CANLib::SocketCanBus canBus;
  CANLib::CanFrame can_tx_frame_;
  CANLib::CanFrame can_rx_frame_;
  std::string can_interface;

  // Per-joint parameters (read from XACRO)
  std::vector<int> joint_node_ids;
  std::vector<int> joint_gear_ratios;
  std::vector<bool> joint_inverted;

  // Control mode
  enum class integration_level_t : std::uint8_t
  {
    UNDEFINED = 0,
    POSITION = 1,
    VELOCITY = 2,
  };

  std::vector<integration_level_t> control_level_;

  // Joint type
  enum class joint_type_t : std::uint8_t
  {
    REVOLUTE = 0,
    PRISMATIC = 1,
  };

  std::vector<joint_type_t> joint_type_;

  // CAN Command Bytes (full byte values, used as: CMD + port_id)
  static constexpr uint8_t PCB_HEARTBEAT_CMD        = 0x10;
  static constexpr uint8_t LED_STATUS_CMD            = 0x11;
  static constexpr uint8_t ABSOLUTE_POS_CONTROL_CMD  = 0x20;
  static constexpr uint8_t VELOCITY_CONTROL_CMD      = 0x30;
  static constexpr uint8_t MOTOR_STATE_CMD           = 0x40;
  static constexpr uint8_t MOTOR_STATUS_CMD          = 0x50;
  static constexpr uint8_t MAINTENANCE_CMD           = 0x60;
  static constexpr uint8_t DC_SPECS_CMD              = 0x70;
};

}  // namespace dc_ros2_control

#endif  // DC_HARDWARE_INTERFACE_HPP_