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
    for (const auto& limb : limb_names_){
        driver_->enableLimbMotors(limb, false);
    }
}

hardware_interface::CallbackReturn ArhaHardwareInterface::on_init(const hardware_interface::HardwareInfo& info){
    
}


}