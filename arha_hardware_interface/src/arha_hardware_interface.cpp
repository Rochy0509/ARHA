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
        for (const auto& limb : limb_names_) {
            driver_->enableLimbMotors(limb, false);
        }
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
    arha_tcp_driver::DriverConfig config;
    config.ip_address = info.hardware_parameters.at("ip_address");
    config.port = std::stoi(info.hardware_parameters.at("port"));

    //creating driver_ with config
    driver_ = std::make_unique<arha_tcp_driver::arhaTCPDriver>(config);

    //Registering each limb
    for (const auto& limb : limb_names_){
        driver_->registerLimb({limb, motor_ids_[limb]});
    }

    stop_polling_ = false;
    return hardware_interface::CallbackReturn::SUCCESS;
}


}