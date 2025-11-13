// Copyright (c) 2025, UMDLoop
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "athena_science_controllers/athena_science_manual.hpp"
#include "athena_science_controllers/athena_science_manual_parameters.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"

namespace
{
// Utility: reset joystick msg for RT buffer
void reset_controller_reference_msg(
  std::shared_ptr<sensor_msgs::msg::Joy> & msg, const std::vector<std::string> & /*joint_names*/)
{
  msg->buttons.assign(16, 0); // or match actual number of buttons
  msg->axes.assign(6, 0.0);   // match actual number of axes
}
}  // namespace

namespace athena_science_controllers
{

AthenaScienceManual::AthenaScienceManual() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn AthenaScienceManual::on_init()
{
  control_mode_.initRT(control_mode_type::STAGE1);

  try {
    param_listener_ = std::make_shared<athena_science_manual::ParamListener>(get_node());
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during controller init: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn AthenaScienceManual::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  // Set default STAGE 1
  current_mode_ = control_mode_type::STAGE1;

  // Fill composite joint groups
  stepper_joints_ = {
    params_.stepper_motor_a,
    params_.stepper_motor_b
  };

  talon_joints_ = {
    params_.talon_lift[0],  // or flatten the array
    params_.talon_scoop
  };

  servo_joints_ = params_.scoop_servos;
  servo_joints_.push_back(params_.auger);
  servo_joints_.push_back(params_.cap);

  auger_spinner_ = params_.auger_spinner;

  if (!params_.state_joints.empty()) {
    state_joints_ = params_.state_joints;
  } else {
    state_joints_ = params_.joints;
  }

  if (params_.joints.size() != state_joints_.size()) {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of 'joints' (%zu) and 'state_joints' (%zu) must be equal!",
      params_.joints.size(), state_joints_.size());
    return CallbackReturn::FAILURE;
  }

  prev_buttons_.assign(12, 0);

  // QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference subscriber
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "/science_manual", subscribers_qos,
    std::bind(&AthenaScienceManual::reference_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg, params_.joints);
  input_ref_.writeFromNonRT(msg);

  // State publisher
  s_publisher_ = get_node()->create_publisher<ControllerStateMsg>(
    "/athena_science_manual/state", rclcpp::QoS(rclcpp::KeepLast(1)));
  state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);

  if (state_publisher_ && state_publisher_->trylock()) {
    state_publisher_->msg_.header.frame_id =
      (!params_.joints.empty() ? params_.joints[0] : std::string("base"));
    state_publisher_->msg_.set_point = 0.0;
    state_publisher_->msg_.process_value = 0.0;
    state_publisher_->msg_.command = 0.0;
    state_publisher_->unlockAndPublish();
  }

  RCLCPP_INFO(get_node()->get_logger(), "Configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

void AthenaScienceManual::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  if (prev_buttons_.empty()) {
    prev_buttons_.resize(msg->buttons.size(), 0);
  }
  input_ref_.writeFromNonRT(msg);
}

controller_interface::InterfaceConfiguration AthenaScienceManual::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : stepper_joints_)
  {
    cfg.names.push_back(joint + "/position");
  }

  // Talons: control velocity
  for (const auto & joint : talon_joints_)
  {
    cfg.names.push_back(joint + "/velocity");
  }

  // Servos (scoop + cap): control position
  for (const auto & joint : servo_joints_)
  {
    cfg.names.push_back(joint + "/position");
  }

  // Auger spinner (if separate): velocity control
  cfg.names.push_back(auger_spinner_ + "/velocity");
  return cfg;
}

controller_interface::InterfaceConfiguration AthenaScienceManual::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : stepper_joints_)
  {
    cfg.names.push_back(joint + "/position");
    cfg.names.push_back(joint + "/velocity");
  }

  for (const auto & joint : talon_joints_)
  {
    cfg.names.push_back(joint + "/velocity");
    cfg.names.push_back(joint + "/position");
  }

  for (const auto & joint : servo_joints_)
  {
    cfg.names.push_back(joint + "/position");
  }

  cfg.names.push_back(auger_spinner_ + "/velocity");
  cfg.names.push_back(auger_spinner_ + "/position");
  return cfg;
}

controller_interface::CallbackReturn AthenaScienceManual::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reset_controller_reference_msg(*(input_ref_.readFromRT)(), params_.joints);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AthenaScienceManual::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  stepper_joints_.clear();
  talon_joints_.clear();
  servo_joints_.clear();
  state_joints_.clear();

  RCLCPP_INFO(get_node()->get_logger(), "Manual controller deactivated and released interfaces");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type AthenaScienceManual::update(
    const rclcpp::Time & /*time*/, 
    const rclcpp::Duration & /*period*/)
{
  
  auto current_ref = input_ref_.readFromRT();

  if (!(*current_ref)) {
    return controller_interface::return_type::OK;
  }

  auto msg = *current_ref;  // shared_ptr<sensor_msgs::msg::Joy>

  int stage_idx = static_cast<int>(current_mode_);  // corresponds to STAGE1..STAGE4

  // Lift Talon
  double lift_cmd = (msg->buttons.size() > 0 && msg->buttons[0]) ? params_.velocity_limits_talon_lift[stage_idx] : 0.0;

  // Stepper motors command
  double stepper_cmd = (msg->buttons.size() > 2 && msg->buttons[2]) ? 
    params_.velocity_limits_stepper[stage_idx] : 0.0;

  // Scoop Talon
  double scoop_cmd = (msg->buttons.size() > 4 && msg->buttons[4]) ? 
    params_.velocity_limits_talon_scoop[stage_idx] : 0.0;

  // Auger
  double auger_cmd = (msg->buttons.size() > 5 && msg->buttons[5]) ? 
    params_.velocity_limits_auger[stage_idx] : 0.0;

  // Auger Spinner
  double auger_spinner_cmd = (msg->buttons.size() > 6 && msg->buttons[6]) ? 
    params_.velocity_limits_auger_spinner[stage_idx] : 0.0;

  scoop_position = std::clamp(scoop_position, 0.0, 1.0);
  auger_position = std::clamp(auger_position, 0.0, 1.0);
  cap_position   = std::clamp(cap_position, 0.0, 1.0);

  // Send velocity commands
  command_interfaces_[IDX_LIFT_TALON_VELOCITY].set_value(lift_cmd);
  command_interfaces_[IDX_STEPPERS_VELOCITY_START].set_value(stepper_cmd);
  command_interfaces_[IDX_SCOOP_TALON_VELOCITY].set_value(scoop_cmd);
  command_interfaces_[IDX_AUGER_VELOCITY].set_value(auger_cmd);

  // --- Send position commands
  command_interfaces_[IDX_SCOOP_SERVO_POSITION].set_value(scoop_position);
  command_interfaces_[IDX_AUGER_SERVO_POSITION].set_value(auger_position);
  command_interfaces_[IDX_CAP_SERVO_POSITION].set_value(cap_position);

  reset_controller_reference_msg(*(input_ref_.readFromRT)(), params_.joints);

  for (const auto &joint_name : params_.joints) {
    if (state_publisher_ && state_publisher_->trylock()) {
      state_publisher_->msg_.header.stamp = get_node()->get_clock()->now();
      state_publisher_->msg_.header.frame_id = joint_name;
      state_publisher_->msg_.set_point = stepper_cmd;
      state_publisher_->msg_.process_value = auger_cmd;
      state_publisher_->msg_.command = lift_cmd;
      state_publisher_->unlockAndPublish();
    }
  }
  
  return controller_interface::return_type::OK;
}


}  // namespace athena_science_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  athena_science_controllers::AthenaScienceManual, controller_interface::ControllerInterface)