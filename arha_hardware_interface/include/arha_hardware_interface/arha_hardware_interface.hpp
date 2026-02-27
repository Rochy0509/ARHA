#ifndef ARHA_HARDWARE_INTERFACE_HPP
#define ARHA_HARDWARE_INTERFACE_HPP

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include "arha_hardware_interface/visibility_control.hpp"

#include <arha_tcp.hpp>
#include <thread>
#include <atomic>
#include <vector>
#include <string>
#include <map>
#include <mutex>
#include <cmath>
#include <fstream>
#include <exception>
#include <algorithm>

namespace arha_hardware_interface {

class ArhaHardwareInterface : public hardware_interface::SystemInterface{
    public:
        ArhaHardwareInterface() = default;
        ~ArhaHardwareInterface() override;

        // Standard lifecycle methods for ros2_control
        CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State& prev_state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State& prev_state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State& prev_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State& prev_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev_state) override;
        CallbackReturn on_error(const rclcpp_lifecycle::State& prev_state) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        ARHA_HARDWARE_PUBLIC
        hardware_interface::return_type prepare_command_mode_switch(
            const std::vector<std::string>& start_interfaces,
            const std::vector<std::string>& stop_interfaces) override;

        ARHA_HARDWARE_PUBLIC
        hardware_interface::return_type perform_command_mode_switch(
            const std::vector<std::string>& start_interfaces,
            const std::vector<std::string>& stop_interfaces) override;

        ARHA_HARDWARE_PUBLIC
        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

        ARHA_HARDWARE_PUBLIC
        hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    protected:
        static rclcpp::Logger getLogger();

        // Multi-limb support
        size_t num_joints_{0};
        std::vector<std::string> limb_names_;
        std::map<std::string, std::vector<std::string>> joint_names_;
        std::map<std::string, std::vector<uint32_t>> motor_ids_;

        // Real-time safe buffers for state and command, per limb
        std::map<std::string, realtime_tools::RealtimeBuffer<std::vector<double>>> position_state_buffer_;
        std::map<std::string, realtime_tools::RealtimeBuffer<std::vector<double>>> velocity_state_buffer_;
        std::map<std::string, realtime_tools::RealtimeBuffer<std::vector<double>>> effort_state_buffer_;

        double timeout_sec_{0.0};

        // Communication driver
        arha_tcp_driver::DriverConfig driver_config_;
        std::unique_ptr<arha_tcp_driver::arhaTCPDriver> driver_;

        // Async polling thread
        std::thread polling_thread_;
        std::atomic<bool> stop_polling_{false};

        void pollingLoop();
        
        // Flat arrays for ros2_control to point to, as RealTimeBuffer lacks stable pointers
        std::vector<double> hw_position_commands_;
        std::vector<double> hw_velocity_commands_;
        std::vector<double> hw_effort_commands_;     
        std::vector<double> hw_position_states_;     
        std::vector<double> hw_velocity_states_;  
        std::vector<double> hw_effort_states_;
        std::vector<double> directions_;
        std::mutex commands_mutex_;
        
        // Flags to check if interfaces are running
        std::atomic<bool> position_interface_running_{false};
        std::atomic<bool> velocity_interface_running_{false};
        std::atomic<bool> effort_interface_running_{false};
        bool zero_on_startup_{false};
};
}


#endif // ARHA_HARDWARE_INTERFACE_HPP