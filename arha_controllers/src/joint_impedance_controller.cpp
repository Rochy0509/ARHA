#include "arha_controllers/joint_impedance_controller.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

namespace arha_controllers
{

controller_interface::CallbackReturn JointImpedanceController::on_init()
{
  try
  {
    auto node = get_node();
    if (!node->has_parameter("joints")) {
      node->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>());
    }
    if (!node->has_parameter("kp")) {
      node->declare_parameter<std::vector<double>>("kp", std::vector<double>());
    }
    if (!node->has_parameter("kd")) {
      node->declare_parameter<std::vector<double>>("kd", std::vector<double>());
    }
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointImpedanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  joint_names_ = node->get_parameter("joints").as_string_array();

  if (joint_names_.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  kp_ = node->get_parameter("kp").as_double_array();
  kd_ = node->get_parameter("kd").as_double_array();

  if (kp_.size() != joint_names_.size() || kd_.size() != joint_names_.size())
  {
    RCLCPP_ERROR(node->get_logger(), "Kp or Kd size mismatch with joints");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Initialize command buffer
  auto initial_commands = std::make_shared<Commands>();
  initial_commands->positions.assign(joint_names_.size(), 0.0);
  initial_commands->velocities.assign(joint_names_.size(), 0.0);
  rt_command_ptr_.writeFromNonRT(initial_commands);

  // Subscriber
  command_subscriber_ = node->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
    "~/command", rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<trajectory_msgs::msg::JointTrajectoryPoint> msg)
    {
      auto command = std::make_shared<Commands>();
      command->positions = msg->positions;
      command->velocities = msg->velocities;
      
      // Pad with zeros if short
      if (command->positions.size() < joint_names_.size()) {
          command->positions.resize(joint_names_.size(), 0.0);
      }
      if (command->velocities.size() < joint_names_.size()) {
          command->velocities.resize(joint_names_.size(), 0.0);
      }
      
      rt_command_ptr_.writeFromNonRT(command);
    });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration JointImpedanceController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_)
  {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

controller_interface::InterfaceConfiguration JointImpedanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_)
  {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return config;
}

controller_interface::CallbackReturn JointImpedanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Clear handles
  joint_state_pos_handles_.clear();
  joint_state_vel_handles_.clear();
  joint_command_eff_handles_.clear();

  // Get state handles
  for (const auto & joint : joint_names_)
  {
    auto pos_handle = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(),
      [&joint](const auto & interface)
      {
        return interface.get_prefix_name() == joint &&
               interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
      });
    if (pos_handle == state_interfaces_.end()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Unable to find position interface for %s", joint.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }
    joint_state_pos_handles_.emplace_back(*pos_handle);

    auto vel_handle = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(),
      [&joint](const auto & interface)
      {
        return interface.get_prefix_name() == joint &&
               interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
      });
    if (vel_handle == state_interfaces_.end()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Unable to find velocity interface for %s", joint.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }
    joint_state_vel_handles_.emplace_back(*vel_handle);
  }

  // Get command handles
  for (const auto & joint : joint_names_)
  {
    auto eff_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&joint](const auto & interface)
      {
        return interface.get_prefix_name() == joint &&
               interface.get_interface_name() == hardware_interface::HW_IF_EFFORT;
      });
    if (eff_handle == command_interfaces_.end()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Unable to find effort interface for %s", joint.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }
    joint_command_eff_handles_.emplace_back(*eff_handle);
  }

  // Initialize command to current position to avoid jumps
  auto initial_commands = std::make_shared<Commands>();
  initial_commands->positions.resize(joint_names_.size());
  initial_commands->velocities.assign(joint_names_.size(), 0.0);
  for (size_t i = 0; i < joint_names_.size(); ++i) {
      // Safely check for uninitialized NaN values from hardware interface and default them to 0
      double val = joint_state_pos_handles_[i].get().get_value();
      initial_commands->positions[i] = std::isnan(val) ? 0.0 : val;
  }
  rt_command_ptr_.writeFromNonRT(initial_commands);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type JointImpedanceController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto commanded_ptr = rt_command_ptr_.readFromRT();
  if (!commanded_ptr || !(*commanded_ptr))
  {
    return controller_interface::return_type::OK;
  }

  bool should_log = (update_counter_++ % 1000 == 0); // Log every 1 second at 1000Hz


  const auto & positions_des = (*commanded_ptr)->positions;
  const auto & velocities_des = (*commanded_ptr)->velocities;

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    const double q_curr = joint_state_pos_handles_[i].get().get_value();
    const double dq_curr = joint_state_vel_handles_[i].get().get_value();
    
    // Default target to current position if command is not active or empty
    double q_des = std::isnan(q_curr) ? 0.0 : q_curr;
    double dq_des = 0.0;
    
    if (i < positions_des.size() && !std::isnan(positions_des[i])) q_des = positions_des[i];
    if (i < velocities_des.size() && !std::isnan(velocities_des[i])) dq_des = velocities_des[i];

    // Read current state correctly, default to 0 if hardware gave NaN
    double q_curr_safe = std::isnan(q_curr) ? q_des : q_curr;
    double dq_curr_safe = std::isnan(dq_curr) ? 0.0 : dq_curr;

    // Control Law: tau = Kp * (q_des - q_curr_safe) + Kd * (dq_des - dq_curr_safe)
    double tau = kp_[i] * (q_des - q_curr_safe) + kd_[i] * (dq_des - dq_curr_safe);

    // Hard limit on torque to prevent erratic behavior / explosions
    const double MAX_TAU = 10.0; // +/- 10 Nm max limit per joint initially
    tau = std::clamp(tau, -MAX_TAU, MAX_TAU);

    if (should_log) {
      RCLCPP_INFO(get_node()->get_logger(), 
        "[%s] Joint: %s | Q_curr: %.3f | Q_des: %.3f | V_curr: %.3f | Tau: %.3f", 
        get_node()->get_name(), joint_names_[i].c_str(), q_curr, q_des, dq_curr, tau);
    }

    joint_command_eff_handles_[i].get().set_value(tau);

  }

  return controller_interface::return_type::OK;
}

}  // namespace arha_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  arha_controllers::JointImpedanceController, controller_interface::ControllerInterface)
