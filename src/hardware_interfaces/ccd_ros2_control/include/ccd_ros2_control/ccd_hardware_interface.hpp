#ifndef CCD_ROS2_CONTROL__CCD_HARDWARE_INTERFACE_HPP_
#define CCD_ROS2_CONTROL__CCD_HARDWARE_INTERFACE_HPP_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "umdloop_can_library/CanFrame.hpp"
#include "umdloop_can_library/SocketCanBus.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "raman_msgs/msg/raman_spectrum.hpp"

namespace CANLib
{
struct CanFrame;
}

namespace ccd_ros2_control
{

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
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  void logger_function();

  // Mirrors servo's DecodedCommand for maintenance frame unpacking
  struct DecodedCommand
  {
    uint8_t command_id;
    std::vector<uint8_t> u8_data;
    std::vector<int16_t> i16_data;
    std::vector<int32_t> i32_data;
  };

  struct CCDJoint
  {
    std::string name;
    uint32_t can_id;

    // State interfaces
    double is_connected;
    double command_success;
    double acquisition_in_progress;
    double data_ready;
    double frames_received;
    double last_frame_id;
    double status;

    // Command interfaces
    double capture_binary_cmd;
    double capture_byte_cmd;
    double status_request;
    double maintenance_request;
    double maintenance_frame_high;
    double maintenance_frame_low;
    double maintenance_data_count;

    // Tracking
    double prev_status_request;
    double prev_maintenance_request;
    double elapsed_status_request_time;
    double elapsed_maintenance_request_time;

    // Internal acquisition state
    bool waiting_for_binary_data;
    bool waiting_for_byte_data;
    double maintenance_frame;
    std::vector<uint8_t> decoded_maintenance_frame;
    std::vector<uint8_t> binary_pixels;
    std::vector<uint8_t> byte_pixels;
  };

private:
  void onCanMessage(const CANLib::CanFrame & frame);
  bool format_maintenance_command(
    CANLib::CanFrame & frame, uint32_t can_id, const DecodedCommand & decoded_cmd);

  std::string can_interface_;
  CANLib::SocketCanBus canBus_;
  CANLib::CanFrame can_tx_frame_;
  bool can_connected_;

  int update_rate_;
  int logger_rate_;
  int logger_state_;
  double elapsed_time_;
  double elapsed_logger_time_;

  rclcpp::Node::SharedPtr pixel_pub_node_;
  rclcpp::Publisher<raman_msgs::msg::RamanSpectrum>::SharedPtr pixel_publisher_;

  std::vector<CCDJoint> CCDJoints_;

  static constexpr uint8_t CMD_REQUEST_BINARY = 0x20;
  static constexpr uint8_t CMD_REQUEST_BYTE   = 0x30;
  static constexpr uint8_t CONFIRM_SEND       = 0x01;

  // CAN response arbitration IDs per protocol
  static constexpr uint32_t CAN_RESP_ACK_ID    = 0x101;
  static constexpr uint32_t CAN_RESP_BINARY_ID = 0x102;
  static constexpr uint32_t CAN_RESP_BYTE_ID   = 0x103;
  static constexpr uint32_t CAN_PIXEL_PUB_ID = 0x104;

  inline DecodedCommand unpack_command_full(int32_t counts_in, int64_t payload_in)
  {
    const uint32_t counts  = static_cast<uint32_t>(counts_in);
    const uint64_t payload = static_cast<uint64_t>(payload_in);

    const uint8_t u8_count  = static_cast<uint8_t>((counts >> 16) & 0xFF);
    const uint8_t i16_count = static_cast<uint8_t>((counts >> 8)  & 0xFF);
    const uint8_t i32_count = static_cast<uint8_t>( counts        & 0xFF);

    int bit_offset = 64;
    auto pop_bits = [&](int bits) -> uint64_t {
      bit_offset -= bits;
      const uint64_t mask = (bits == 64)
        ? std::numeric_limits<uint64_t>::max()
        : ((1ULL << bits) - 1ULL);
      return (payload >> bit_offset) & mask;
    };

    DecodedCommand result{};
    result.command_id = static_cast<uint8_t>(pop_bits(8));
    for (uint8_t i = 0; i < u8_count;  ++i)
      result.u8_data.push_back(static_cast<uint8_t>(pop_bits(8)));
    for (uint8_t i = 0; i < i16_count; ++i)
      result.i16_data.push_back(static_cast<int16_t>(static_cast<uint16_t>(pop_bits(16))));
    for (uint8_t i = 0; i < i32_count; ++i)
      result.i32_data.push_back(static_cast<int32_t>(static_cast<uint32_t>(pop_bits(32))));
    return result;
  }
};

}  // namespace ccd_ros2_control

#endif  // CCD_ROS2_CONTROL__CCD_HARDWARE_INTERFACE_HPP_