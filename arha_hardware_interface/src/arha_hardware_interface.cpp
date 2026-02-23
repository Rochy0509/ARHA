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

    if (info.joints.size() != 12) {
        RCLCPP_FATAL(getLogger(), "Expected 12 joints but got %zu.", info.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }
    
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
    
    //stoping polling thread
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
}