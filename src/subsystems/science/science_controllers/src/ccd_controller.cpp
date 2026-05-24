#include "science_controllers/ccd_controller.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace science_controllers
{

CCDSnapshotController::CCDSnapshotController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn CCDSnapshotController::on_init()
{
  try {
    auto_declare<std::string>("ccd_name", "spectrometry_ccd");
    auto_declare<std::vector<std::string>>("command_interfaces", {"capture_byte"});
    auto_declare<std::vector<std::string>>(
      "state_interfaces",
      {
        "is_connected",
        "command_success",
        "acquisition_in_progress",
        "data_ready",
        "frames_received",
        "last_frame_id"
      });

    auto_declare<std::string>("snapshot_service_name", "~/request_snapshot");
    auto_declare<std::string>("status_publish_topic", "~/status");
    auto_declare<std::string>("snapshot_publish_topic", "/raman/raw_spectrum");

    auto_declare<std::string>("spectrometer_id", "pda_spectrometer");
    auto_declare<double>("integration_time_ms", 100.0);
    auto_declare<double>("laser_wavelength_nm", 785.0);
    auto_declare<int>("num_photodiodes", 3648);
    auto_declare<double>("wavenumber_min", 200.0);
    auto_declare<double>("wavenumber_max", 3500.0);

    auto_declare<double>("publish_rate", 10.0);
    auto_declare<double>("acquisition_timeout_sec", 5.0);
    auto_declare<int>("expected_total_frames", 609);
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception during CCDSnapshotController on_init: %s\n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  snapshot_requested_ = false;
  snapshot_in_progress_ = false;
  snapshot_complete_published_ = false;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CCDSnapshotController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & interface_name : command_interface_names_) {
    cfg.names.push_back(ccd_name_ + "/" + interface_name);
  }

  return cfg;
}

controller_interface::InterfaceConfiguration
CCDSnapshotController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & interface_name : state_interface_names_) {
    cfg.names.push_back(ccd_name_ + "/" + interface_name);
  }

  return cfg;
}

controller_interface::CallbackReturn CCDSnapshotController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  ccd_name_ = get_node()->get_parameter("ccd_name").as_string();
  command_interface_names_ = get_node()->get_parameter("command_interfaces").as_string_array();
  state_interface_names_ = get_node()->get_parameter("state_interfaces").as_string_array();

  snapshot_service_name_ = get_node()->get_parameter("snapshot_service_name").as_string();
  status_publish_topic_ = get_node()->get_parameter("status_publish_topic").as_string();
  snapshot_publish_topic_ = get_node()->get_parameter("snapshot_publish_topic").as_string();

  spectrometer_id_ = get_node()->get_parameter("spectrometer_id").as_string();
  integration_time_ms_ = get_node()->get_parameter("integration_time_ms").as_double();
  laser_wavelength_nm_ = get_node()->get_parameter("laser_wavelength_nm").as_double();
  wavenumber_min_ = get_node()->get_parameter("wavenumber_min").as_double();
  wavenumber_max_ = get_node()->get_parameter("wavenumber_max").as_double();

  publish_rate_ = get_node()->get_parameter("publish_rate").as_double();
  acquisition_timeout_sec_ = get_node()->get_parameter("acquisition_timeout_sec").as_double();

  num_photodiodes_ = static_cast<int>(get_node()->get_parameter("num_photodiodes").as_int());
  expected_total_frames_ = static_cast<int>(get_node()->get_parameter("expected_total_frames").as_int());

  if (command_interface_names_.size() != CMD_ITFS_COUNT) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu command interface, got %zu",
      static_cast<size_t>(CMD_ITFS_COUNT),
      command_interface_names_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (state_interface_names_.size() != STATE_ITFS_COUNT) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu state interfaces, got %zu",
      static_cast<size_t>(STATE_ITFS_COUNT),
      state_interface_names_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  wavenumber_axis_ = make_wavenumber_axis();

  snapshot_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
    snapshot_service_name_,
    std::bind(
      &CCDSnapshotController::handle_snapshot_request,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  pixel_subscriber_ = get_node()->create_subscription<msgs::msg::RamanSpectrum>(
    "/raman/raw_pixels",
    rclcpp::QoS(1).reliable(),
    [this](const msgs::msg::RamanSpectrum::SharedPtr msg) {
      pixel_buffer_.writeFromNonRT(msg->intensities);
  });

  status_publisher_ = get_node()->create_publisher<StatusMsg>(
    status_publish_topic_, rclcpp::QoS(10));

  realtime_status_publisher_ = std::make_unique<StatusPublisher>(status_publisher_);

  spectrum_publisher_ = get_node()->create_publisher<msgs::msg::RamanSpectrum>(
    snapshot_publish_topic_, rclcpp::QoS(10));

  auto initial_request = std::make_shared<SnapshotRequest>();
  initial_request->requested = false;
  snapshot_request_buffer_.writeFromNonRT(initial_request);

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured CCDSnapshotController for HWI '%s', publishing RamanSpectrum on '%s'",
    ccd_name_.c_str(),
    snapshot_publish_topic_.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CCDSnapshotController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (command_interfaces_.size() != CMD_ITFS_COUNT) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu command interfaces, got %zu",
      static_cast<size_t>(CMD_ITFS_COUNT),
      command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (state_interfaces_.size() != STATE_ITFS_COUNT) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu state interfaces, got %zu",
      static_cast<size_t>(STATE_ITFS_COUNT),
      state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  command_interfaces_[CMD_CAPTURE_BYTE].set_value(0.0);

  snapshot_requested_ = false;
  snapshot_in_progress_ = false;
  snapshot_complete_published_ = false;
  acquisition_start_time_ = get_node()->now();
  last_status_publish_time_ = get_node()->now();

  RCLCPP_INFO(get_node()->get_logger(), "CCD snapshot controller activated");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CCDSnapshotController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(std::numeric_limits<double>::quiet_NaN());
  }

  snapshot_requested_ = false;
  snapshot_in_progress_ = false;
  snapshot_complete_published_ = false;

  RCLCPP_INFO(get_node()->get_logger(), "CCD snapshot controller deactivated");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CCDSnapshotController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & /*period*/)
{
  auto request_ptr = snapshot_request_buffer_.readFromRT();

  if (request_ptr && *request_ptr && (*request_ptr)->requested && !snapshot_requested_ &&
      !snapshot_in_progress_)
  {
    snapshot_requested_ = true;
    snapshot_in_progress_ = true;
    snapshot_complete_published_ = false;
    acquisition_start_time_ = time;

    command_interfaces_[CMD_CAPTURE_BYTE].set_value(1.0);

    auto consumed_request = std::make_shared<SnapshotRequest>();
    consumed_request->requested = false;
    snapshot_request_buffer_.writeFromNonRT(consumed_request);

    RCLCPP_INFO(get_node()->get_logger(), "CCD snapshot request sent");
  } else {
    command_interfaces_[CMD_CAPTURE_BYTE].set_value(0.0);
  }

  if (snapshot_in_progress_) {
    if (get_state_bool(STATE_DATA_READY) ||
      get_state_double(STATE_FRAMES_RECEIVED) >= static_cast<double>(expected_total_frames_ - 1)) {
      if (!snapshot_complete_published_) {
        if (!pixel_data_ready_) {
          pixel_data_ready_ = true;
          return controller_interface::return_type::OK;
        }

        publish_spectrum(time);
        publish_status(time);
        snapshot_complete_published_ = true;
        pixel_data_ready_ = false;
      }

      snapshot_requested_ = false;
      snapshot_in_progress_ = false;
    } else if (acquisition_timed_out(time)) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "CCD snapshot timed out after %.2f seconds. Frames received: %.0f",
        acquisition_timeout_sec_,
        get_state_double(STATE_FRAMES_RECEIVED));

      snapshot_requested_ = false;
      snapshot_in_progress_ = false;
      snapshot_complete_published_ = false;
      publish_status(time);
    }
  }

  const double publish_period = publish_rate_ > 0.0 ? 1.0 / publish_rate_ : 1.0;
  if ((time - last_status_publish_time_).seconds() >= publish_period) {
    publish_status(time);
    last_status_publish_time_ = time;
  }

  return controller_interface::return_type::OK;
}

void CCDSnapshotController::handle_snapshot_request(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (snapshot_in_progress_) {
    response->success = false;
    response->message = "CCD snapshot already in progress";
    return;
  }

  if (!get_state_bool(STATE_IS_CONNECTED)) {
    response->success = false;
    response->message = "CCD HWI is not connected";
    return;
  }

  auto request_msg = std::make_shared<SnapshotRequest>();
  request_msg->requested = true;
  snapshot_request_buffer_.writeFromNonRT(request_msg);

  response->success = true;
  response->message = "CCD snapshot request accepted";
}

void CCDSnapshotController::publish_status(const rclcpp::Time & /*time*/)
{
  if (!realtime_status_publisher_ || !realtime_status_publisher_->trylock()) {
    return;
  }

  std::ostringstream ss;
  ss << "ccd_name=" << ccd_name_
     << ", connected=" << get_state_double(STATE_IS_CONNECTED)
     << ", command_success=" << get_state_double(STATE_COMMAND_SUCCESS)
     << ", acquisition_in_progress=" << get_state_double(STATE_ACQUISITION_IN_PROGRESS)
     << ", data_ready=" << get_state_double(STATE_DATA_READY)
     << ", frames_received=" << get_state_double(STATE_FRAMES_RECEIVED)
     << ", last_frame_id=" << get_state_double(STATE_LAST_FRAME_ID)
     << ", expected_total_frames=" << expected_total_frames_;

  realtime_status_publisher_->msg_.data = ss.str();
  realtime_status_publisher_->unlockAndPublish();
}

void CCDSnapshotController::publish_spectrum(const rclcpp::Time & time)
{
  auto msg = msgs::msg::RamanSpectrum();

  msg.header.stamp = time;
  msg.header.frame_id = "raman_probe";
  msg.spectrometer_id = spectrometer_id_;
  msg.integration_time_ms = integration_time_ms_;
  msg.laser_wavelength_nm = laser_wavelength_nm_;
  msg.accumulations = 1;

  msg.wavenumber_axis = wavenumber_axis_;
  msg.intensities = get_latest_intensities();

  spectrum_publisher_->publish(msg);

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Published RamanSpectrum snapshot: %zu intensities, %.0f frames received",
    msg.intensities.size(),
    get_state_double(STATE_FRAMES_RECEIVED));
}

bool CCDSnapshotController::acquisition_timed_out(const rclcpp::Time & time) const
{
  return snapshot_in_progress_ &&
         ((time - acquisition_start_time_).seconds() > acquisition_timeout_sec_);
}

bool CCDSnapshotController::get_state_bool(size_t index) const
{
  return get_state_double(index) > 0.5;
}

double CCDSnapshotController::get_state_double(size_t index) const
{
  if (index >= state_interfaces_.size()) {
    return 0.0;
  }
  return state_interfaces_[index].get_value();
}

std::vector<double> CCDSnapshotController::make_wavenumber_axis() const
{
  std::vector<double> axis;

  if (num_photodiodes_ <= 0) {
    return axis;
  }

  axis.resize(static_cast<size_t>(num_photodiodes_));

  if (num_photodiodes_ == 1) {
    axis[0] = wavenumber_min_;
    return axis;
  }

  const double step =
    (wavenumber_max_ - wavenumber_min_) / static_cast<double>(num_photodiodes_ - 1);

  for (int i = 0; i < num_photodiodes_; ++i) {
    axis[static_cast<size_t>(i)] = wavenumber_min_ + step * static_cast<double>(i);
  }

  return axis;
}

std::vector<double> CCDSnapshotController::get_latest_intensities()
{
  auto buffer_ptr = pixel_buffer_.readFromRT();
  if (buffer_ptr && !(*buffer_ptr).empty()) {
    return *buffer_ptr;
  }
  // Fallback to zeros if buffer not yet populated
  return std::vector<double>(static_cast<size_t>(num_photodiodes_), 0.0);
}

}  // namespace science_controllers

PLUGINLIB_EXPORT_CLASS(
  science_controllers::CCDSnapshotController,
  controller_interface::ControllerInterface)