#include "arha_hardware_interface/arha_hardware_interface.hpp"

namespace arha_hardware_interface{
    
rclcpp::Logger ArhaHardwareInterface::getLogger() {
    return rclcpp::get_logger("Arha Hardware Interface");
}

ArhaHardwareInterface::~ArhaHardwareInterface() {
    stop_polling_ = true;
    if (polling_thread_.joinable()){
        polling_thread_.join();
    }

    //safeguard in case driver_ is null so it skip motor disable
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
    
    //for loop to extract information from URDF to set the joints to the limb so it match ARHA_TCP_DRIVER CONFIG
    for (const auto& joint : info.joints){

        std::string limb = joint.parameters.at("limb_name"); // get what limb is the joint assigned to
        std::string motor_id = joint.parameters.at("motor_id");

        if (std::find(limb_names_.begin(), limb_names_.end(), limb) == limb_names_.end() ){
            limb_names_.push_back(limb);
        }

        joint_names_[limb].push_back(joint.name); //append the joint_name to its limb's vector 
        motor_ids_[limb].push_back(std::stoul(motor_id)); //append motor_id to the limb's joints
    }

    //for loop to initialize interfaces
    for (const auto& limb : limb_names_){
        auto num_joints = joint_names_[limb].size();

        //state buffer initialization
        position_state_buffer_[limb].writeFromNonRT(std::vector<double>(num_joints, 0.0));
        velocity_state_buffer_[limb].writeFromNonRT(std::vector<double>(num_joints, 0.0));
        effort_state_buffer_[limb].writeFromNonRT(std::vector<double>(num_joints, 0.0));

        //command buffer initialization
        position_command_buffer_[limb].writeFromNonRT(std::vector<double>(num_joints, 0.0));
        velocity_command_buffer_[limb].writeFromNonRT(std::vector<double>(num_joints, 0.0));
        effort_command_buffer_[limb].writeFromNonRT(std::vector<double>(num_joints, 0.0));
    }

    hw_position_commands_.resize(num_joints_, 0.0);
    hw_velocity_commands_.resize(num_joints_, 0.0);
    hw_effort_commands_.resize(num_joints_, 0.0);

    hw_position_states_.resize(num_joints_, 0.0);
    hw_velocity_states_.resize(num_joints_, 0.0);
    hw_effort_states_.resize(num_joints_, 0.0);


    // extracting connection settings for ARHA_TCP_DRIVER from URDF
    driver_config_.ip_address = info.hardware_parameters.at("ip_address");
    driver_config_.port = std::stoi(info.hardware_parameters.at("port"));

    stop_polling_ = false;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArhaHardwareInterface::on_configure(const rclcpp_lifecycle::State& /*prev_state*/){
    
    //creating driver_ with config
    driver_ = std::make_unique<arha_tcp_driver::arhaTCPDriver>(driver_config_);

    //Registering each limb
    for (const auto& limb : limb_names_){
        driver_->registerLimb({limb, motor_ids_[limb]});
    }

    //starting connection
    auto err = driver_->connect();

    //checking if the stm32 was not reached to start connection
    if (err != arha_tcp_driver::DriverError::SUCCESS){
        RCLCPP_FATAL(getLogger(), "Failed to connect to STM32: %s", 
        driver_->getLastErrorMessage().c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }
    else {
        RCLCPP_INFO(getLogger(), "Connected to STM32 at %s:%d", 
        driver_config_.ip_address.c_str(), driver_config_.port);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArhaHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*prev_state*/){
    RCLCPP_INFO(getLogger(), "Activating ARHA hardware");

    auto err = driver_->enableMotors(true);
    
    //check for any error
    if (err != arha_tcp_driver::DriverError::SUCCESS){
        RCLCPP_FATAL(getLogger(), "Failed to enable motors: %s",
        driver_->getLastErrorMessage().c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }
    else {
        RCLCPP_INFO(getLogger(), "Motors Enabled!");
    }

    //start polling_thread
    stop_polling_ = false;
    polling_thread_ = std::thread(&ArhaHardwareInterface::pollingLoop, this);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArhaHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*prev_state*/){
    
    //stopping polling thread
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

    //stopping polling thread
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
    
    //checking that driver is disable
    if (driver_) {
        driver_->disconnect();
        driver_.reset(); 
    }
    RCLCPP_INFO(getLogger(), "Cleanup complete");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArhaHardwareInterface::on_error(const rclcpp_lifecycle::State& /*prev_state*/){
   
    //stopping polling thread
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
}