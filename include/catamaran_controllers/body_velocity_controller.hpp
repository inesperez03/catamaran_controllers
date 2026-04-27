#pragma once

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sura_msgs/msg/navigator.hpp"

namespace catamaran_controllers
{

class BodyVelocityController : public controller_interface::ChainableControllerInterface
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

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  bool on_set_chained_mode(bool chained_mode) override;

  controller_interface::return_type update_reference_from_subscribers() override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  using TwistMsg = geometry_msgs::msg::Twist;
  using NavigatorMsg = sura_msgs::msg::Navigator;

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & params);

  rclcpp::Subscription<TwistMsg>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<NavigatorMsg>::SharedPtr navigator_sub_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<TwistMsg>> cmd_vel_buffer_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<NavigatorMsg>> navigator_buffer_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  std::string cmd_vel_topic_{"/cmd_vel"};
  std::string navigator_topic_{"/navigator_msg"};
  std::string body_force_controller_name_{"body_force_controller"};

  std::vector<std::string> reference_interface_names_;

  double kp_u_{0.0};
  double ki_u_{0.0};
  double kd_u_{0.0};
  double integral_limit_u_{0.0};
  double max_force_x_{0.0};

  double kp_r_{0.0};
  double ki_r_{0.0};
  double kd_r_{0.0};
  double integral_limit_r_{0.0};

  double integral_u_{0.0};
  double integral_r_{0.0};

  double prev_error_u_{0.0};
  double prev_error_r_{0.0};

  bool first_update_{true};
};

}  // namespace catamaran_controllers
