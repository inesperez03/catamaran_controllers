#include "catamaran_controllers/body_position_controller.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace catamaran_controllers
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

}  // namespace

double BodyPositionController::normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double BodyPositionController::clampAbs(double value, double limit)
{
  if (limit <= 0.0) {
    return value;
  }
  return std::clamp(value, -limit, limit);
}

controller_interface::CallbackReturn BodyPositionController::on_init()
{
  try {
    auto_declare<std::string>("navigator_topic", "/navigator_msg");
    auto_declare<std::string>("setpoint_topic", "/body_position/setpoint");
    auto_declare<std::string>("body_velocity_controller_name", "body_velocity_controller");

    auto_declare<double>("kp_position", 0.8);
    auto_declare<double>("kp_yaw", 1.8);
    auto_declare<double>("max_linear_speed", 0.35);
    auto_declare<double>("max_angular_speed", 0.5);
    auto_declare<double>("position_hold_radius", 0.25);
    auto_declare<double>("yaw_tolerance", 0.12);
    auto_declare<double>("slow_down_radius", 1.2);
    auto_declare<double>("heading_error_stop", 1.2);
    auto_declare<double>("reverse_distance_threshold", 0.8);
    auto_declare<double>("max_reverse_speed", 0.10);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception in on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
BodyPositionController::command_interface_configuration() const
{
  const std::string prefix = body_velocity_controller_name_;

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {
      prefix + "/linear.x",
      prefix + "/angular.z"
    }
  };
}

controller_interface::InterfaceConfiguration
BodyPositionController::state_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::NONE
  };
}

controller_interface::CallbackReturn BodyPositionController::on_configure(
  const rclcpp_lifecycle::State &)
{
  navigator_topic_ = get_node()->get_parameter("navigator_topic").as_string();
  setpoint_topic_ = get_node()->get_parameter("setpoint_topic").as_string();
  body_velocity_controller_name_ =
    get_node()->get_parameter("body_velocity_controller_name").as_string();

  kp_position_ = get_node()->get_parameter("kp_position").as_double();
  kp_yaw_ = get_node()->get_parameter("kp_yaw").as_double();
  max_linear_speed_ = get_node()->get_parameter("max_linear_speed").as_double();
  max_angular_speed_ = get_node()->get_parameter("max_angular_speed").as_double();
  position_hold_radius_ = get_node()->get_parameter("position_hold_radius").as_double();
  yaw_tolerance_ = get_node()->get_parameter("yaw_tolerance").as_double();
  slow_down_radius_ = get_node()->get_parameter("slow_down_radius").as_double();
  heading_error_stop_ = get_node()->get_parameter("heading_error_stop").as_double();
  reverse_distance_threshold_ = get_node()->get_parameter("reverse_distance_threshold").as_double();
  max_reverse_speed_ = get_node()->get_parameter("max_reverse_speed").as_double();

  navigator_sub_ = get_node()->create_subscription<NavigatorMsg>(
    navigator_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const NavigatorMsg::SharedPtr msg)
    {
      navigator_buffer_.writeFromNonRT(msg);
    });

  setpoint_sub_ = get_node()->create_subscription<SetPointMsg>(
    setpoint_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const SetPointMsg::SharedPtr msg)
    {
      setpoint_buffer_.writeFromNonRT(msg);
    });

  RCLCPP_INFO(get_node()->get_logger(), "Configured BodyPositionController");
  RCLCPP_INFO(get_node()->get_logger(), "navigator topic: %s", navigator_topic_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "setpoint topic: %s", setpoint_topic_.c_str());
  RCLCPP_INFO(
    get_node()->get_logger(),
    "body_velocity_controller_name: %s",
    body_velocity_controller_name_.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BodyPositionController::on_activate(
  const rclcpp_lifecycle::State &)
{
  last_setpoint_msg_ptr_ = nullptr;
  has_active_target_ = false;
  holding_current_position_ = false;

  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BodyPositionController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  last_setpoint_msg_ptr_ = nullptr;
  has_active_target_ = false;
  holding_current_position_ = false;

  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type BodyPositionController::update(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  auto navigator_msg = navigator_buffer_.readFromRT();
  auto setpoint_msg = setpoint_buffer_.readFromRT();

  if (!navigator_msg || !(*navigator_msg)) {
    return controller_interface::return_type::OK;
  }

  if (command_interfaces_.size() != 2) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "Expected 2 command interfaces, got %zu",
      command_interfaces_.size());
    return controller_interface::return_type::ERROR;
  }

  const double x = (*navigator_msg)->position.position.x;
  const double y = (*navigator_msg)->position.position.y;
  const double yaw = (*navigator_msg)->rpy.z;

  if (setpoint_msg && *setpoint_msg) {
    const SetPointMsg * current_setpoint_ptr = (*setpoint_msg).get();
    if (current_setpoint_ptr != last_setpoint_msg_ptr_) {
      last_setpoint_msg_ptr_ = current_setpoint_ptr;
      active_target_.x = (*setpoint_msg)->position.x;
      active_target_.y = (*setpoint_msg)->position.y;
      active_target_.yaw = (*setpoint_msg)->rpy.z;
      has_active_target_ = true;
      holding_current_position_ = false;
    }
  }

  if (!has_active_target_) {
    active_target_.x = x;
    active_target_.y = y;
    active_target_.yaw = yaw;
    has_active_target_ = true;
    holding_current_position_ = true;
  }

  const double x_ref = active_target_.x;
  const double y_ref = active_target_.y;
  const double yaw_ref = active_target_.yaw;

  const double dx = x_ref - x;
  const double dy = y_ref - y;
  const double distance = std::hypot(dx, dy);

  const double desired_heading = std::atan2(dy, dx);
  const double heading_error = normalizeAngle(desired_heading - yaw);
  const double final_yaw_error = normalizeAngle(yaw_ref - yaw);

  double linear_ref = 0.0;
  double angular_ref = 0.0;

  if (distance <= position_hold_radius_) {
    active_target_.x = x;
    active_target_.y = y;
    holding_current_position_ = true;
  }

  if (distance > position_hold_radius_) {
    const double distance_for_gain =
      slow_down_radius_ > 0.0 ? std::min(distance, slow_down_radius_) : distance;
    const bool use_reverse =
      reverse_distance_threshold_ > 0.0 &&
      distance <= reverse_distance_threshold_ &&
      std::abs(heading_error) > (kPi / 2.0);

    const double motion_heading_error = use_reverse
      ? normalizeAngle(heading_error - std::copysign(kPi, heading_error))
      : heading_error;
    const double heading_weight = std::max(0.0, std::cos(motion_heading_error));
    const double speed_limit = use_reverse ? max_reverse_speed_ : max_linear_speed_;
    const double direction = use_reverse ? -1.0 : 1.0;
    linear_ref = direction * clampAbs(
      kp_position_ * distance_for_gain * heading_weight,
      speed_limit);

    angular_ref = clampAbs(kp_yaw_ * motion_heading_error, max_angular_speed_);
  } else {
    if (std::abs(final_yaw_error) > yaw_tolerance_) {
      angular_ref = clampAbs(kp_yaw_ * final_yaw_error, max_angular_speed_);
    }
  }

  command_interfaces_[0].set_value(linear_ref);
  command_interfaces_[1].set_value(angular_ref);

  RCLCPP_INFO_THROTTLE(
    get_node()->get_logger(),
    *get_node()->get_clock(),
    1000,
    "target=(%.2f, %.2f) pos=(%.2f, %.2f) dist=%.3f radius=%.3f hold=%s yaw_ref=%.2f yaw=%.2f u_ref=%.3f r_ref=%.3f",
    active_target_.x, active_target_.y, x, y, distance, position_hold_radius_,
    holding_current_position_ ? "true" : "false",
    yaw_ref, yaw, linear_ref, angular_ref);

  return controller_interface::return_type::OK;
}

}  // namespace catamaran_controllers

PLUGINLIB_EXPORT_CLASS(
  catamaran_controllers::BodyPositionController,
  controller_interface::ControllerInterface)
