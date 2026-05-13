// Copyright (c) 2025, UMDLoop
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

//
// Source of this file are templates in
// [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
//

#ifndef SCIENCE_CONTROLLERS__CCD_SNAPSHOT_CONTROLLER_HPP_
#define SCIENCE_CONTROLLERS__CCD_SNAPSHOT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "rclcpp_lifecycle/state.hpp"
#include "science_controllers/visibility_control.h"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/string.hpp"

// Replace later with your real custom messages
#include "raman_msgs/msg/raman_spectrum.hpp"


namespace science_controllers
{

// name constants for state interfaces
static constexpr size_t STATE_MY_ITFS = 0;

// name constants for command interfaces
static constexpr size_t CMD_MY_ITFS = 0;



class CCDSnapshotController : public controller_interface::ControllerInterface
{
public:
  SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC
  CCDSnapshotController();

  SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  SCIENCE_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  using ControllerModeSrvType = std_srvs::srv::SetBool;



protected:

  void handle_snapshot_request(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void publish_status(const rclcpp::Time & time);
  void publish_spectrum(const rclcpp::Time & time);
  bool acquisition_timed_out(const rclcpp::Time & time) const;

  bool get_state_bool(size_t index) const;
  double get_state_double(size_t index) const;

  std::vector<double> make_wavenumber_axis() const;
  std::vector<double> get_latest_intensities() const;

  std::string ccd_name_;
  std::vector<std::string> command_interface_names_;
  std::vector<std::string> state_interface_names_;
  std::vector<double> wavenumber_axis_;

  std::string snapshot_service_name_;
  std::string status_publish_topic_;
  std::string snapshot_publish_topic_;

  std::string spectrometer_id_;
  double integration_time_ms_;
  double laser_wavelength_nm_;
  int num_photodiodes_;
  double wavenumber_min_;
  double wavenumber_max_;
  
  double publish_rate_;
  double acquisition_timeout_sec_;
  int expected_total_frames_;


  struct SnapshotRequest
  {
    bool requested = false; 
  };

  // Service
  realtime_tools::RealtimeBuffer<std::shared_ptr<SnapshotRequest>> snapshot_request_buffer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr snapshot_service_;

  // Publishers
  using StatusMsg = std_msgs::msg::String;
  using StatusPublisher = realtime_tools::RealtimePublisher<StatusMsg>;

  rclcpp::Publisher<StatusMsg>::SharedPtr status_publisher_;
  std::unique_ptr<StatusPublisher> realtime_status_publisher_;

  rclcpp::Publisher<raman_msgs::msg::RamanSpectrum>::SharedPtr spectrum_publisher_;

  // Subscribers
  rclcpp::Subscription<raman_msgs::msg::RamanSpectrum>::SharedPtr pixel_subscriber_;
  realtime_tools::RealtimeBuffer<std::vector<double>> pixel_buffer_;

  bool snapshot_requested_;
  bool snapshot_in_progress_;
  bool snapshot_complete_published_;
  rclcpp::Time acquisition_start_time_;
  rclcpp::Time last_status_publish_time_;

private:
  // callback for topic interface

  enum CommandInterfaces
  {
    CMD_CAPTURE_BYTE = 0,

    // Total number of interfaces
    CMD_ITFS_COUNT
  };

  enum StateInterfaces
  {
    STATE_IS_CONNECTED = 0,
    STATE_COMMAND_SUCCESS,
    STATE_ACQUISITION_IN_PROGRESS,
    STATE_DATA_READY,
    STATE_FRAMES_RECEIVED,
    STATE_LAST_FRAME_ID,
    STATE_ITFS_COUNT
  };

  
};

} // namespace science_controllers


#endif  // SCIENCE_CONTROLLERS__CCDSNAPSHOT_CONTROLLER_HPP_
