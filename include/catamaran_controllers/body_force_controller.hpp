#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "controller_interface/chainable_controller_interface.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "urdf/model.h"

namespace catamaran_controllers
{

class BodyForceController : public controller_interface::ChainableControllerInterface
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
  static Eigen::Isometry3d urdfPoseToEigen(const urdf::Pose & pose);

  Eigen::Isometry3d jointPoseInBase(
    const urdf::Model & model,
    const std::string & joint_name,
    const std::string & base_link) const;

  bool buildThrusterAllocationMatrix(
    const urdf::Model & model,
    const std::string & base_link,
    const std::vector<std::string> & thruster_joints);

  Eigen::MatrixXd pseudoInverse(
    const Eigen::MatrixXd & matrix,
    double tolerance = 1e-6) const;

  using WrenchMsg = geometry_msgs::msg::Wrench;

  rclcpp::Subscription<WrenchMsg>::SharedPtr body_force_sub_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<WrenchMsg>> rt_buffer_ptr_;

  std::string input_topic_;
  std::string base_link_;
  std::vector<std::string> thruster_joints_;

  std::vector<std::string> reference_interface_names_;

  Eigen::MatrixXd thruster_allocation_matrix_;
};

}  // namespace catamaran_controllers