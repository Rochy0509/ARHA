#include "arha_hardware_interface/arha_hardware_interface.hpp"
#include <pthread.h>

namespace arha_hardware_interface{
    
rclcpp::Logger ArhaHardwareInterface::getLogger() {
    return rclcpp::get_logger("Arha Hardware Interface");
}

ArhaHardwareInterface::~ArhaHardwareInterface() {
    stop_polling_ = true;
    if (polling_thread_.joinable()){
        polling_thread_.join();
    }

    // Safeguards in case driver_ is null so it skips motor disable
    if (driver_) {
        driver_->enableMotors(false);
    }
}

hardware_interface::CallbackReturn ArhaHardwareInterface::on_init(const hardware_interface::HardwareInfo& info){
    
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (info.joints.empty()) {
        RCLCPP_FATAL(getLogger(), "No joints found in URDF!");
        return hardware_interface::CallbackReturn::ERROR;
    }

    num_joints_ = info.joints.size();
    
    // Extracts information from URDF to assign joints to the limb
    for (const auto& joint : info.joints){
        std::string limb = joint.parameters.at("limb_name"); // get what limb is the joint assigned to
        std::string motor_id = joint.parameters.at("motor_id");

        if (std::find(limb_names_.begin(), limb_names_.end(), limb) == limb_names_.end() ){
            limb_names_.push_back(limb);
        }

        joint_names_[limb].push_back(joint.name);
        motor_ids_[limb].push_back(std::stoul(motor_id));

        double dir = 1.0;
        if (joint.parameters.count("direction")) {
            dir = (std::stod(joint.parameters.at("direction")) < 0.0) ? -1.0 : 1.0;
        }
        directions_.push_back(dir);
    }

    // Initializes interfaces
    for (const auto& limb : limb_names_){
        auto num_joints = joint_names_[limb].size();

        // Initializes state buffers
        position_state_buffer_[limb].writeFromNonRT(std::vector<double>(num_joints, 0.0));
        velocity_state_buffer_[limb].writeFromNonRT(std::vector<double>(num_joints, 0.0));
        effort_state_buffer_[limb].writeFromNonRT(std::vector<double>(num_joints, 0.0));
    }

    hw_position_commands_.resize(num_joints_, 0.0);
    hw_velocity_commands_.resize(num_joints_, 0.0);
    hw_effort_commands_.resize(num_joints_, 0.0);

    hw_position_states_.resize(num_joints_, 0.0);
    hw_velocity_states_.resize(num_joints_, 0.0);
    hw_effort_states_.resize(num_joints_, 0.0);


    // Connection settings from URDF
    driver_config_.ip_address = info.hardware_parameters.at("ip_address");
    driver_config_.port = std::stoi(info.hardware_parameters.at("port"));
    driver_config_.socket_timeout_ms = 5000; // Multi-second timeout tolerates heavy ROS 2 FastDDS PointCloud multicasts
    driver_config_.verbose = true;          // Enables verbose to debug byte stream

    // Optional zero on startup
    zero_on_startup_ = false;
    if (info.hardware_parameters.count("zero_on_startup")) {
        zero_on_startup_ = (info.hardware_parameters.at("zero_on_startup") == "true");
    }

    stop_polling_ = false;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArhaHardwareInterface::on_configure(const rclcpp_lifecycle::State& /*prev_state*/){
    
    // Creates driver_ with config
    driver_ = std::make_unique<arha_tcp_driver::arhaTCPDriver>(driver_config_);

    // Registers each limb
    for (const auto& limb : limb_names_){
        driver_->registerLimb({limb, motor_ids_[limb]});
    }

    // Starts connection
    auto err = driver_->connect();

    // Checks if the stm32 was reached to start connection
    if (err != arha_tcp_driver::DriverError::SUCCESS){
        RCLCPP_FATAL(getLogger(), "Failed to connect to STM32: %s", 
        driver_->getLastErrorMessage().c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(getLogger(), "Connected to STM32 at %s:%d", 
        driver_config_.ip_address.c_str(), driver_config_.port);

    // Zeroes encoders if requested
    if (zero_on_startup_) {
        RCLCPP_INFO(getLogger(), "Zeroing motor encoders...");
        for (const auto& limb : limb_names_) {
            auto z_err = driver_->setEncoderZero(limb);
            if (z_err != arha_tcp_driver::DriverError::SUCCESS) {
                RCLCPP_WARN(getLogger(), "Failed to zero %s: %s",
                    limb.c_str(), driver_->getLastErrorMessage().c_str());
            } else {
                RCLCPP_INFO(getLogger(), "Zeroed %s encoders", limb.c_str());
            }
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArhaHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*prev_state*/){
    RCLCPP_INFO(getLogger(), "Activating ARHA hardware");

    auto err = driver_->enableMotors(true);
    // Checks for any error
    if (err != arha_tcp_driver::DriverError::SUCCESS){
        RCLCPP_ERROR(getLogger(), "Failed to enable motors: %s. Continuing anyway to allow debug.",
        driver_->getLastErrorMessage().c_str());
    }
    else {
        RCLCPP_INFO(getLogger(), "Motors Enabled!");
        // Waits 0.5s for STM32 to power up drivers
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    // Starts polling_thread
    stop_polling_ = false;
    if (polling_thread_.joinable()){
        stop_polling_ = true;
        polling_thread_.join();
        stop_polling_ = false;
    }
    polling_thread_ = std::thread(&ArhaHardwareInterface::pollingLoop, this);

    // Sets Real-Time Priority to prevent Linux CFS from starving the driver
    struct sched_param param;
    param.sched_priority = 80; // High real-time priority
    if (pthread_setschedparam(polling_thread_.native_handle(), SCHED_FIFO, &param) != 0) {
        RCLCPP_WARN(getLogger(), "Failed to set RT priority. Run with sudo or setcap cap_sys_nice.");
    } else {
        RCLCPP_INFO(getLogger(), "Polling thread set to SCHED_FIFO priority 80");
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArhaHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*prev_state*/){
    
    // Stops polling thread
    stop_polling_ = true;
    if (polling_thread_.joinable()){
        polling_thread_.join();
    }
    
    //disabling all motors
    auto err = driver_->enableMotors(false);

    //check for errors
    if (err != arha_tcp_driver::DriverError::SUCCESS){
        RCLCPP_FATAL(getLogger(), "Failed to disable all motors: %s",
        driver_->getLastErrorMessage().c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }
    else {
        RCLCPP_INFO(getLogger(), "Motors disabled!");
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArhaHardwareInterface::on_shutdown(const rclcpp_lifecycle::State& /*prev_state*/){

    // Stops polling thread
    stop_polling_ = true;
    if (polling_thread_.joinable()){
        polling_thread_.join();
    }

    if (driver_) {
        auto err = driver_->enableMotors(false);
        if (err != arha_tcp_driver::DriverError::SUCCESS){
            RCLCPP_FATAL(getLogger(), "Failed to shutdown all motors: %s",
                driver_->getLastErrorMessage().c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        driver_->disconnect();
        RCLCPP_INFO(getLogger(), "Motors shutdown and disconnected successfully!");
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArhaHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& /*prev_state*/){
    
    // Checks that driver is disabled
    if (driver_) {
        driver_->disconnect();
        driver_.reset(); 
    }
    RCLCPP_INFO(getLogger(), "Cleanup complete");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArhaHardwareInterface::on_error(const rclcpp_lifecycle::State& /*prev_state*/){
   
    // Stops polling thread
    stop_polling_ = true;
    if (polling_thread_.joinable()){
        polling_thread_.join();
    }

    if (driver_) {
        auto err = driver_->enableMotors(false);
        if (err != arha_tcp_driver::DriverError::SUCCESS){
            RCLCPP_WARN(getLogger(), "Failed to disable motors during error handling: %s",
                driver_->getLastErrorMessage().c_str());
        }
        driver_->disconnect();
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> ArhaHardwareInterface::export_command_interfaces() {
    
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    size_t i = 0;
    for (const auto& limb_name : limb_names_){
        for (const auto& joint_name : joint_names_[limb_name]){
            command_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_POSITION, &hw_position_commands_[i]);
            command_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_commands_[i]);
            command_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_EFFORT, &hw_effort_commands_[i]);
            i++;
        }
    }

    return command_interfaces;
}

std::vector<hardware_interface::StateInterface> ArhaHardwareInterface::export_state_interfaces() {
    
    std::vector<hardware_interface::StateInterface> state_interfaces;

    size_t i = 0;
    for (const auto& limb_name : limb_names_){
        for (const auto& joint_name : joint_names_[limb_name]){
            state_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_POSITION, &hw_position_states_[i]);
            state_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_states_[i]);
            state_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_EFFORT, &hw_effort_states_[i]);
            i++;
        }
    }

    return state_interfaces;
}

hardware_interface::return_type ArhaHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces [[maybe_unused]]){
        
    std::string requested_mode;
    for (const auto& interface : start_interfaces){
        
        size_t pos = interface.find('/');
        if (pos != std::string::npos){
            std::string mode = interface.substr(pos + 1);
            if (requested_mode.empty()){
                requested_mode = mode;
            }
            else if (mode != requested_mode){
                RCLCPP_ERROR(getLogger(), "Modes are incorrect in start_interfaces");
                return hardware_interface::return_type::ERROR;
            }
        }
    }
    if (!start_interfaces.empty() && start_interfaces.size() != num_joints_ ) {
        RCLCPP_ERROR(getLogger(), "Expected %zu interfaces for mode switch, got %zu.", num_joints_,
        start_interfaces.size());
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArhaHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces [[maybe_unused]]){
    
    position_interface_running_ = false;
    velocity_interface_running_ = false;
    effort_interface_running_ = false;

    if (!start_interfaces.empty()){
        std::string mode = start_interfaces[0].substr(start_interfaces[0].find('/') + 1);
        if (mode == hardware_interface::HW_IF_POSITION) {
            position_interface_running_ = true;
            RCLCPP_INFO(getLogger(), "Position mode enabled for all joints");
        }
        else if (mode == hardware_interface::HW_IF_VELOCITY) {
            velocity_interface_running_ = true;
            RCLCPP_INFO(getLogger(), "Velocity mode enabled for all joints");
        }
        else if (mode == hardware_interface::HW_IF_EFFORT) {
            effort_interface_running_ = true;
            RCLCPP_INFO(getLogger(), "Effort mode enabled for all joints");
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArhaHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {

    size_t index = 0;
    for (const auto& limb : limb_names_) {
        const auto* positions  = position_state_buffer_[limb].readFromRT();
        const auto* velocities = velocity_state_buffer_[limb].readFromRT();
        const auto* efforts    = effort_state_buffer_[limb].readFromRT();

        for (size_t j = 0; j < positions->size(); ++j) {
            double dir = directions_[index];
            hw_position_states_[index] = dir * (*positions)[j];
            hw_velocity_states_[index] = dir * (*velocities)[j];
            hw_effort_states_[index]   = dir * (*efforts)[j];
            index++;
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArhaHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    // Commands are directly written to hw_*_commands_ by ros2_control.
    // pollingLoop reads from them directly.
    return hardware_interface::return_type::OK;
}

void ArhaHardwareInterface::pollingLoop() {
    constexpr auto cycle_time = std::chrono::milliseconds(50); // 20 Hz
    uint32_t debug_counter = 0;

    while (!stop_polling_) {
        auto const now = std::chrono::steady_clock::now();
        auto const wakeup_time = now + cycle_time;

        if (!driver_->isConnected()) {
            static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
            RCLCPP_WARN_THROTTLE(getLogger(), steady_clock, 2000, "TCP disconnected, attempting reconnect...");
            if (driver_->reconnect() != arha_tcp_driver::DriverError::SUCCESS) {
                // Back off for 1 second before trying again so we don't spam the network
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }
            RCLCPP_INFO(getLogger(), "TCP Reconnected successfully.");
            // Specifically does NOT auto-zero encoders here for safety against sagging.
        }
        
        // Logs continuously to debug MoveIt trajectory vs State mismatch
        bool do_log = true; 
        size_t index = 0;

        for (const auto& limb : limb_names_) {
            size_t n = joint_names_[limb].size();

            // ── Send commands ──
            if (position_interface_running_) {
                std::vector<double> cmds(n);
                {
                    std::lock_guard<std::mutex> lock(commands_mutex_);
                    for (size_t j = 0; j < n; ++j) {
                        cmds[j] = directions_[index + j] * hw_position_commands_[index + j];
                    }
                }
                
                static std::map<std::string, std::vector<double>> last_cmds;
                bool should_send = false;
                if (last_cmds.find(limb) == last_cmds.end()) {
                    last_cmds[limb] = std::vector<double>(n, 0.0);
                    should_send = true;
                } else {
                    for (size_t j = 0; j < n; ++j) {
                        if (std::abs(cmds[j] - last_cmds[limb][j]) > 1e-4) {
                            should_send = true;
                            break;
                        }
                    }
                }

                if (should_send) {
                    last_cmds[limb] = cmds;
                    if (do_log && !cmds.empty()) {
                        std::string s = "[DEBUG] SET_POS " + limb + ":";
                        for (size_t j = 0; j < cmds.size(); ++j)
                            s += " " + std::to_string(cmds[j]);
                        RCLCPP_INFO(getLogger(), "%s", s.c_str());
                    }
                    auto set_err = driver_->setPositions(limb, cmds);
                    if (set_err != arha_tcp_driver::DriverError::SUCCESS) {
                        if (do_log) {
                            RCLCPP_ERROR(getLogger(), "setPositions FAILED for %s: %s",
                                limb.c_str(), driver_->getLastErrorMessage().c_str());
                        }
                        if (set_err == arha_tcp_driver::DriverError::TIMEOUT || 
                            set_err == arha_tcp_driver::DriverError::RECEIVE_FAILED || 
                            set_err == arha_tcp_driver::DriverError::SEND_FAILED) {
                            driver_->disconnect();
                        }
                    }
                }
            } else if (velocity_interface_running_) {
                std::vector<double> cmds(n);
                {
                    std::lock_guard<std::mutex> lock(commands_mutex_);
                    for (size_t j = 0; j < n; ++j) {
                        cmds[j] = directions_[index + j] * hw_velocity_commands_[index + j];
                    }
                }
                driver_->setVelocities(limb, cmds);
            } else if (effort_interface_running_) {
                std::vector<double> cmds(n);
                {
                    std::lock_guard<std::mutex> lock(commands_mutex_);
                    for (size_t j = 0; j < n; ++j) {
                        cmds[j] = directions_[index + j] * hw_effort_commands_[index + j];
                    }
                }
                driver_->setEfforts(limb, cmds);
            } else if (do_log) {
                RCLCPP_WARN(getLogger(), "No command interface active for %s", limb.c_str());
            }
            
            // ── Read states ──
            std::vector<double> positions(n);
            std::vector<double> velocities(n);
            std::vector<double> efforts(n);

            auto err = driver_->getStates(limb, positions, velocities, efforts);
            if (err == arha_tcp_driver::DriverError::SUCCESS) {
                if (do_log) {
                    std::string s = "[DEBUG] STATE " + limb + ":";
                    for (size_t j = 0; j < n; ++j)
                        s += " p=" + std::to_string(positions[j]);
                    RCLCPP_INFO(getLogger(), "%s", s.c_str());
                }
                position_state_buffer_[limb].writeFromNonRT(positions);
                velocity_state_buffer_[limb].writeFromNonRT(velocities);
                effort_state_buffer_[limb].writeFromNonRT(efforts);
            } else {
                if (do_log) {
                    RCLCPP_WARN(getLogger(), "getStates failed for %s: %s",
                        limb.c_str(), driver_->getLastErrorMessage().c_str());
                }
                if (err == arha_tcp_driver::DriverError::TIMEOUT || 
                    err == arha_tcp_driver::DriverError::RECEIVE_FAILED || 
                    err == arha_tcp_driver::DriverError::SEND_FAILED) {
                    driver_->disconnect();
                }
            }

            index += n;
        }

        debug_counter++;
        std::this_thread::sleep_until(wakeup_time);
    }
}

} //namespace arha_hardware_interface

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(arha_hardware_interface::ArhaHardwareInterface, hardware_interface::SystemInterface)