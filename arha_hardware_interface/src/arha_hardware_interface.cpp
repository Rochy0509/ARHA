#include "arha_hardware_interface/arha_hardware_interface.hpp"
#include <pthread.h>

#include <math.h>
#include <string.h>
#include <sstream>
#include <iomanip>
#include <limits>

namespace arha_hardware_interface{

// Torque constants (Nm/A) for MyActuator motors
static const std::map<uint32_t, double> motor_torque_constants = {
    {1, 7.46}, // X10S2V3
    {2, 7.50}, // X8-60
    {3, 1.92}, // X8-20
    {4, 7.50}, // X8-60
    {5, 1.92}, // X8-20
    {6, 1.25}  // X6-8
};
    
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


    // Optional zero on startup
    zero_on_startup_ = false;
    if (info.hardware_parameters.count("zero_on_startup")) {
        zero_on_startup_ = (info.hardware_parameters.at("zero_on_startup") == "true");
    }

    // Connection settings from URDF
    driver_config_.ip_address = info.hardware_parameters.at("ip_address");
    driver_config_.port = std::stoi(info.hardware_parameters.at("port"));
    // Multi-second timeout tolerates heavy ROS 2 FastDDS PointCloud multicasts
    // But zeroing the motors requires an extremely long timeout (up to 45 seconds)
    driver_config_.socket_timeout_ms = zero_on_startup_ ? 45000 : 5000; 
    driver_config_.verbose = false;          // Disabled verbose byte stream debugging

    // Gravity compensation settings
    gravity_compensation_enabled_ = false;
    if (info.hardware_parameters.count("gravity_compensation")) {
        gravity_compensation_enabled_ = (info.hardware_parameters.at("gravity_compensation") == "true");
    }


    if (gravity_compensation_enabled_) {
        RCLCPP_INFO(getLogger(), "Gravity compensation enabled");
        for (const auto& limb : limb_names_) {
            std::string base_param = limb + "_base_link";
            std::string tip_param = limb + "_tip_link";
            if (info.hardware_parameters.count(base_param)) {
                base_links_[limb] = info.hardware_parameters.at(base_param);
            } else {
                base_links_[limb] = "base_link"; // Default
            }
            if (info.hardware_parameters.count(tip_param)) {
                tip_links_[limb] = info.hardware_parameters.at(tip_param);
            } else {
                RCLCPP_ERROR(getLogger(), "Missing %s for gravity compensation", tip_param.c_str());
                gravity_compensation_enabled_ = false;
            }
        }
    }

    if (gravity_compensation_enabled_) {
        std::string urdf_content;
        if (info.hardware_parameters.count("urdf_path")) {
            std::string urdf_path = info.hardware_parameters.at("urdf_path");
            std::ifstream urdf_file(urdf_path);
            if (urdf_file.is_open()) {
                urdf_content.assign((std::istreambuf_iterator<char>(urdf_file)),
                                    std::istreambuf_iterator<char>());
            } else {
                RCLCPP_ERROR(getLogger(), "Failed to open URDF file at %s", urdf_path.c_str());
                gravity_compensation_enabled_ = false;
            }
        } else {
            RCLCPP_ERROR(getLogger(), "Missing 'urdf_path' parameter for gravity compensation");
            gravity_compensation_enabled_ = false;
        }

        if (gravity_compensation_enabled_) {
            initializeDynamics(urdf_content);
        }
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
    // Allow partial activations (e.g. left_arm_controller activating 6 joints, right_arm_controller activating 6)
    // if (!start_interfaces.empty() && start_interfaces.size() != num_joints_ ) {
    //     RCLCPP_ERROR(getLogger(), "Expected %zu interfaces for mode switch, got %zu.", num_joints_,
    //     start_interfaces.size());
    //     return hardware_interface::return_type::ERROR;
    // }

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
        
        size_t index = 0;

        for (const auto& limb : limb_names_) {
            size_t n = joint_names_[limb].size();

            // ── Read states ──
            std::vector<double> current_pos(n);
            std::vector<double> current_vel(n);
            std::vector<double> current_eff(n);

            auto err = driver_->getStates(limb, current_pos, current_vel, current_eff);
            if (err == arha_tcp_driver::DriverError::SUCCESS) {
                position_state_buffer_[limb].writeFromNonRT(current_pos);
                velocity_state_buffer_[limb].writeFromNonRT(current_vel);
                effort_state_buffer_[limb].writeFromNonRT(current_eff);

                // Updates local states for immediate use in this cycle
                for (size_t j = 0; j < n; ++j) {
                    double dir = directions_[index + j];
                    hw_position_states_[index + j] = dir * current_pos[j];
                    hw_velocity_states_[index + j] = dir * current_vel[j];
                    hw_effort_states_[index + j]   = dir * current_eff[j];
                }

                if (debug_counter % 20 == 0) {
                    std::stringstream ss_raw;
                    ss_raw << "RawPos " << limb << " (rad):";
                    for(size_t j = 0; j < n; ++j) ss_raw << " " << std::fixed << std::setprecision(4) << current_pos[j];
                    RCLCPP_INFO(getLogger(), "%s", ss_raw.str().c_str());
                }
            } else {
                if (debug_counter % 20 == 0) {
                    RCLCPP_WARN(getLogger(), "getStates FAILED for %s (err=%d)", limb.c_str(), static_cast<int>(err));
                }
                if (err == arha_tcp_driver::DriverError::TIMEOUT || 
                    err == arha_tcp_driver::DriverError::RECEIVE_FAILED || 
                    err == arha_tcp_driver::DriverError::SEND_FAILED) {
                    driver_->disconnect();
                }
            }

            // ── Gravity Compensation (computed regardless of control mode) ──
            std::vector<double> gravity_torques(n, 0.0);
            if (gravity_compensation_enabled_ && gravity_solvers_.count(limb)) {
                KDL::JntArray q(chains_[limb].getNrOfJoints());
                KDL::JntArray tau_g(chains_[limb].getNrOfJoints());
                
                for (size_t j = 0; j < n; ++j) {
                    q(j) = hw_position_states_[index + j];
                }
                
                if (gravity_solvers_[limb]->CartToJnt(q, KDL::JntArray(n), KDL::JntArray(n), KDL::Wrenches(chains_[limb].getNrOfSegments(), KDL::Wrench::Zero()), tau_g) >= 0) {
                    for (size_t j = 0; j < n; ++j) {
                        gravity_torques[j] = tau_g(j);
                    }
                }

                if (debug_counter % 20 == 0) { // 1Hz logging
                    std::stringstream ss_q;
                    ss_q << "JointPos " << limb << " (rad):";
                    for(size_t j = 0; j < n; ++j) ss_q << " " << std::fixed << std::setprecision(3) << q(j);
                    RCLCPP_INFO(getLogger(), "%s", ss_q.str().c_str());

                    std::stringstream ss;
                    ss << "GravComp " << limb << " (Nm):";
                    for(auto g : gravity_torques) ss << " " << std::fixed << std::setprecision(2) << g;
                    RCLCPP_INFO(getLogger(), "%s", ss.str().c_str());
                }
            }

            // ── Send commands ──
            if (position_interface_running_) {
                std::vector<double> cmds(n);
                {
                    std::lock_guard<std::mutex> lock(commands_mutex_);
                    for (size_t j = 0; j < n; ++j) {
                        cmds[j] = directions_[index + j] * hw_position_commands_[index + j];
                    }
                }
                
                if (true) {
                    static std::map<std::string, std::vector<double>> last_cmds;
                    bool should_send = false;
                    if (last_cmds.find(limb) == last_cmds.end()) {
                        // Initialize with NaN so the first command is always sent
                        last_cmds[limb] = std::vector<double>(n, std::numeric_limits<double>::quiet_NaN());
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
                        driver_->setPositions(limb, cmds);
                    }
                }

                // Gravity compensation feedforward torques alongside position commands
                if (gravity_compensation_enabled_) {
                    std::vector<double> effort_cmds(n);
                    for (size_t j = 0; j < n; ++j) {
                        double kt = 1.0;
                        uint32_t motor_id = motor_ids_[limb][j];
                        if (motor_torque_constants.count(motor_id)) {
                            kt = motor_torque_constants.at(motor_id);
                        }
                        effort_cmds[j] = directions_[index + j] * (gravity_torques[j] / kt);
                    }
                    driver_->setEfforts(limb, effort_cmds);
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
            } else if (effort_interface_running_ || gravity_compensation_enabled_) {
                std::vector<double> cmds(n);

                {
                    std::lock_guard<std::mutex> lock(commands_mutex_);
                    for (size_t j = 0; j < n; ++j) {
                        double kt = 1.0;
                        uint32_t motor_id = motor_ids_[limb][j];
                        if (motor_torque_constants.count(motor_id)) {
                            kt = motor_torque_constants.at(motor_id);
                        }

                        double control_effort_nm = hw_effort_commands_[index + j];
                        
                        double total_torque_nm = control_effort_nm + gravity_torques[j];
                        
                        // total_torque_nm = std::clamp(total_torque_nm, -MAX_TORQUE_NM, MAX_TORQUE_NM);

                        cmds[j] = directions_[index + j] * (total_torque_nm / kt);
                    }
                }

                driver_->setEfforts(limb, cmds);
            }
 
            index += n;
        }

        debug_counter++;
        std::this_thread::sleep_until(wakeup_time);
    }
}

void ArhaHardwareInterface::initializeDynamics(const std::string& robot_description) {
    if (robot_description.empty()) {
        RCLCPP_ERROR(getLogger(), "Robot description is empty, cannot initialize dynamics");
        gravity_compensation_enabled_ = false;
        return;
    }

    KDL::Tree tree;
    if (!kdl_parser::treeFromString(robot_description, tree)) {
        RCLCPP_ERROR(getLogger(), "Failed to construct KDL tree from robot description");
        gravity_compensation_enabled_ = false;
        return;
    }

    for (const auto& limb : limb_names_) {
        if (chains_.find(limb) == chains_.end()) {
            KDL::Chain chain;
            if (!tree.getChain(base_links_[limb], tip_links_[limb], chain)) {
                RCLCPP_ERROR(getLogger(), "Failed to get KDL chain for %s from %s to %s",
                    limb.c_str(), base_links_[limb].c_str(), tip_links_[limb].c_str());
                continue;
            }
            chains_[limb] = chain;

            // Log total mass for verification
            double total_mass = 0;
            for(size_t i=0; i < chain.getNrOfSegments(); i++) {
                total_mass += chain.getSegment(i).getInertia().getMass();
            }
            RCLCPP_INFO(getLogger(), "KDL chain for %s initialized. Joints: %d, Segments: %d, Total Mass: %.2f kg", 
                limb.c_str(), chain.getNrOfJoints(), chain.getNrOfSegments(), total_mass);

            gravity_solvers_[limb] = std::make_shared<KDL::ChainIdSolver_RNE>(chains_[limb], KDL::Vector(0, 0, -9.81));
            
            // Maps ROS joints to KDL segment indices
            joint_to_kdl_index_[limb].clear();
            for (const auto& joint_name : joint_names_[limb]) {
                bool found = false;
                for (size_t i = 0; i < chains_[limb].getNrOfSegments(); ++i) {
                    if (chains_[limb].getSegment(i).getJoint().getName() == joint_name) {
                        joint_to_kdl_index_[limb].push_back(i);
                        found = true;
                        break;
                    }
                }
                if (!found) {
                     RCLCPP_WARN(getLogger(), "Joint %s not found in KDL chain for %s", joint_name.c_str(), limb.c_str());
                }
            }
        }
    }
}

} //namespace arha_hardware_interface

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(arha_hardware_interface::ArhaHardwareInterface, hardware_interface::SystemInterface)