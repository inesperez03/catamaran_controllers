#pragma once

#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sura_msgs/msg/auv_controller_set_point.hpp"
#include "sura_msgs/msg/navigator.hpp"

namespace catamaran_controllers
{

class BodyPositionController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  using NavigatorMsg = sura_msgs::msg::Navigator;
  using SetPointMsg = sura_msgs::msg::AuvControllerSetPoint;

  struct ActiveTarget
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  static double normalizeAngle(double angle);
  static double clampAbs(double value, double limit);

  rclcpp::Subscription<NavigatorMsg>::SharedPtr navigator_sub_;
  rclcpp::Subscription<SetPointMsg>::SharedPtr setpoint_sub_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<NavigatorMsg>> navigator_buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<SetPointMsg>> setpoint_buffer_;

  std::string navigator_topic_{"/navigator_msg"};
  std::string setpoint_topic_{"/body_position/setpoint"};
  std::string body_velocity_controller_name_{"body_velocity_controller"};

  double kp_position_{0.0};
  double kp_yaw_{0.0};
  double max_linear_speed_{0.0};
  double max_angular_speed_{0.0};
  double position_hold_radius_{0.0};
  double yaw_tolerance_{0.0};
  double slow_down_radius_{0.0};
  double heading_error_stop_{0.0};
  double reverse_distance_threshold_{0.0};
  double max_reverse_speed_{0.0};

  const SetPointMsg * last_setpoint_msg_ptr_{nullptr};
  ActiveTarget active_target_{};
  bool has_active_target_{false};
  bool holding_current_position_{false};
};

}  // namespace catamaran_controllers
