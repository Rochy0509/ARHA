#ifndef ARHA_CONTROLLERS__JOINT_IMPEDANCE_CONTROLLER_HPP_
#define ARHA_CONTROLLERS__JOINT_IMPEDANCE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace arha_controllers
{

class JointImpedanceController : public controller_interface::ControllerInterface
{
public:
  JointImpedanceController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  std::vector<std::string> joint_names_;
  
  // Gains
  std::vector<double> kp_;
  std::vector<double> kd_;

  // Command setpoints (desired position/velocity)
  struct Commands
  {
    std::vector<double> positions;
    std::vector<double> velocities;
  };
  realtime_tools::RealtimeBuffer<std::shared_ptr<Commands>> rt_command_ptr_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr command_subscriber_;

  // Handles
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_state_pos_handles_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_state_vel_handles_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_command_eff_handles_;

  uint64_t update_counter_ = 0;
};

}  // namespace arha_controllers

#endif  // ARHA_CONTROLLERS__JOINT_IMPEDANCE_CONTROLLER_HPP_
