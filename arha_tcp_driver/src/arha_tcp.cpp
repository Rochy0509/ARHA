#include "arha_tcp.hpp"
#include <vector>
#include <cstdint>

void arha_tcp_driver::arhaTCPDriver::encodeUInt8(std::vector<uint8_t>& buffer, uint8_t value) {
    buffer.push_back(value);
}

#include <cmath>
#include <cstring>
#include <chrono>
#include <thread>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <cerrno>
#include <netinet/tcp.h>

namespace arha_tcp_driver {

// Constructor / Destructor

arhaTCPDriver::arhaTCPDriver(const DriverConfig& config)
    : config_(config) {}

arhaTCPDriver::~arhaTCPDriver() {
    disconnect();
}

// Connection

DriverError arhaTCPDriver::connect() {
    std::lock_guard<std::mutex> lock(socket_mutex_);

    if (connected_.load()) {
        return DriverError::SUCCESS;
    }

    for (int attempt = 0; attempt <= config_.max_connection_retries; ++attempt) {
        auto err = createSocket();
        if (err != DriverError::SUCCESS) {
            if (attempt < config_.max_connection_retries) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(config_.connection_retry_delay_ms));
                continue;
            }
            return setLastError(err, "Failed to create socket after retries");
        }

        err = setSocketTimeout(config_.socket_timeout_ms);
        if (err != DriverError::SUCCESS) {
            closeSocket();
            return setLastError(err, "Failed to set socket timeout");
        }

        struct sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(static_cast<uint16_t>(config_.port));

        if (inet_pton(AF_INET, config_.ip_address.c_str(), &server_addr.sin_addr) <= 0) {
            closeSocket();
            return setLastError(DriverError::CONNECTION_FAILED,
                "Invalid IP address: " + config_.ip_address);
        }

        if (::connect(socket_fd_,
                       reinterpret_cast<struct sockaddr*>(&server_addr),
                       sizeof(server_addr)) < 0) {
            closeSocket();
            if (attempt < config_.max_connection_retries) {
                logInfo("Connection attempt " + std::to_string(attempt + 1) +
                        " failed, retrying...");
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(config_.connection_retry_delay_ms));
                continue;
            }
            return setLastError(DriverError::CONNECTION_FAILED,
                "Failed to connect to " + config_.ip_address + ":" +
                std::to_string(config_.port) + " - " + std::strerror(errno));
        }

        connected_.store(true);
        logInfo("Connected to " + config_.ip_address + ":" + std::to_string(config_.port));
        return DriverError::SUCCESS;
    }

    return setLastError(DriverError::CONNECTION_FAILED, "Connection failed after all retries");
}

void arhaTCPDriver::disconnect() {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (connected_.load()) {
        closeSocket();
        connected_.store(false);
        logInfo("Disconnected");
    }
}

bool arhaTCPDriver::isConnected() const {
    return connected_.load();
}

DriverError arhaTCPDriver::reconnect() {
    disconnect();
    return connect();
}

// Registration

DriverError arhaTCPDriver::registerLimb(const LimbConfig& limb) {
    if (limb.name.empty() || limb.motor_ids.empty()) {
        return setLastError(DriverError::INVALID_DATA,
            "Limb name and motor_ids must not be empty");
    }

    limbs_[limb.name] = limb;
    logInfo("Registered limb '" + limb.name + "' with " +
            std::to_string(limb.motor_ids.size()) + " joints");
    return DriverError::SUCCESS;
}

bool arhaTCPDriver::hasLimb(const std::string& limb_name) const {
    return limbs_.count(limb_name) > 0;
}

const LimbConfig* arhaTCPDriver::getLimbConfig(const std::string& limb_name) const {
    auto it = limbs_.find(limb_name);
    if (it == limbs_.end()) return nullptr;
    return &it->second;
}

std::vector<std::string> arhaTCPDriver::getRegisteredLimbs() const {
    std::vector<std::string> names;
    names.reserve(limbs_.size());
    for (const auto& pair : limbs_) {
        names.push_back(pair.first);
    }
    return names;
}

size_t arhaTCPDriver::getTotalJoints() const {
    size_t total = 0;
    for (const auto& pair : limbs_) {
        total += pair.second.motor_ids.size();
    }
    return total;
}

// Batch commands

DriverError arhaTCPDriver::setPositions(const std::string& limb_name,
                                         const std::vector<double>& positions) {
    auto err = validateLimb(limb_name);
    if (err != DriverError::SUCCESS) return err;

    const auto& limb = limbs_.at(limb_name);
    if (positions.size() != limb.motor_ids.size()) {
        return setLastError(DriverError::JOINT_COUNT_MISMATCH,
            "Expected " + std::to_string(limb.motor_ids.size()) +
            " positions for '" + limb_name + "', got " +
            std::to_string(positions.size()));
    }

    std::vector<uint8_t> payload;
    encodeString(payload, limb_name);
    encodeUInt8(payload, static_cast<uint8_t>(limb.motor_ids.size()));
    for (size_t i = 0; i < limb.motor_ids.size(); ++i) {
        encodeUInt32(payload, limb.motor_ids[i]);
        encodeMotorValue(payload, positions[i]);
    }

    std::lock_guard<std::mutex> lock(socket_mutex_);
    return sendAndWaitAck(CommandType::SET_LIMB_POSITIONS, payload);
}

DriverError arhaTCPDriver::setVelocities(const std::string& limb_name,
                                           const std::vector<double>& velocities) {
    auto err = validateLimb(limb_name);
    if (err != DriverError::SUCCESS) return err;

    const auto& limb = limbs_.at(limb_name);
    if (velocities.size() != limb.motor_ids.size()) {
        return setLastError(DriverError::JOINT_COUNT_MISMATCH,
            "Expected " + std::to_string(limb.motor_ids.size()) +
            " velocities for '" + limb_name + "', got " +
            std::to_string(velocities.size()));
    }

    std::vector<uint8_t> payload;
    encodeString(payload, limb_name);
    encodeUInt8(payload, static_cast<uint8_t>(limb.motor_ids.size()));
    for (size_t i = 0; i < limb.motor_ids.size(); ++i) {
        encodeUInt32(payload, limb.motor_ids[i]);
        encodeMotorValue(payload, velocities[i]);
    }

    std::lock_guard<std::mutex> lock(socket_mutex_);
    return sendAndWaitAck(CommandType::SET_LIMB_VELOCITIES, payload);
}

DriverError arhaTCPDriver::setEfforts(const std::string& limb_name,
                                       const std::vector<double>& efforts) {
    auto err = validateLimb(limb_name);
    if (err != DriverError::SUCCESS) return err;

    const auto& limb = limbs_.at(limb_name);
    if (efforts.size() != limb.motor_ids.size()) {
        return setLastError(DriverError::JOINT_COUNT_MISMATCH,
            "Expected " + std::to_string(limb.motor_ids.size()) +
            " efforts for '" + limb_name + "', got " +
            std::to_string(efforts.size()));
    }

    std::vector<uint8_t> payload;
    encodeString(payload, limb_name);
    encodeUInt8(payload, static_cast<uint8_t>(limb.motor_ids.size()));
    for (size_t i = 0; i < limb.motor_ids.size(); ++i) {
        encodeUInt32(payload, limb.motor_ids[i]);
        encodeMotorValue(payload, efforts[i]);
    }

    std::lock_guard<std::mutex> lock(socket_mutex_);
    return sendAndWaitAck(CommandType::SET_LIMB_EFFORTS, payload);
}

DriverError arhaTCPDriver::getStates(const std::string& limb_name,
                                      std::vector<double>& positions,
                                      std::vector<double>& velocities,
                                      std::vector<double>& efforts) {
    auto err = validateLimb(limb_name);
    if (err != DriverError::SUCCESS) return err;

    const auto& limb = limbs_.at(limb_name);
    const size_t num_joints = limb.motor_ids.size();

    std::vector<uint8_t> payload;
    encodeString(payload, limb_name);
    encodeUInt8(payload, static_cast<uint8_t>(num_joints));
    for (auto id : limb.motor_ids) {
        encodeUInt32(payload, id);
    }

    std::lock_guard<std::mutex> lock(socket_mutex_);
    err = sendPacket(CommandType::GET_LIMB_STATES, payload);
    if (err != DriverError::SUCCESS) return err;

    std::vector<uint8_t> response;
    err = receivePacket(response);
    if (err != DriverError::SUCCESS) return err;

    const size_t expected_size = num_joints * 12;
    if (response.size() < expected_size) {
        return setLastError(DriverError::INVALID_DATA,
            "Response too short for " + std::to_string(num_joints) + " joints: got " +
            std::to_string(response.size()) + ", expected " + std::to_string(expected_size));
    }

    positions.resize(num_joints);
    velocities.resize(num_joints);
    efforts.resize(num_joints);

    size_t offset = 0;
    for (size_t i = 0; i < num_joints; ++i) {
        positions[i]  = decodeMotorValue(response, offset); offset += 4;
        velocities[i] = decodeMotorValue(response, offset); offset += 4;
        efforts[i]    = decodeMotorValue(response, offset); offset += 4;
    }

    return DriverError::SUCCESS;
}

// Single-joint commands

DriverError arhaTCPDriver::setPosition(const std::string& limb_name,
                                        size_t joint_index, double position) {
    auto err = validateLimbJoint(limb_name, joint_index);
    if (err != DriverError::SUCCESS) return err;

    uint32_t motor_id = limbs_.at(limb_name).motor_ids[joint_index];

    std::vector<uint8_t> payload;
    encodeString(payload, limb_name);
    encodeUInt32(payload, motor_id);
    encodeMotorValue(payload, position);

    std::lock_guard<std::mutex> lock(socket_mutex_);
    return sendAndWaitAck(CommandType::SET_MOTOR_POSITION, payload);
}

DriverError arhaTCPDriver::setVelocity(const std::string& limb_name,
                                        size_t joint_index, double velocity) {
    auto err = validateLimbJoint(limb_name, joint_index);
    if (err != DriverError::SUCCESS) return err;

    uint32_t motor_id = limbs_.at(limb_name).motor_ids[joint_index];

    std::vector<uint8_t> payload;
    encodeString(payload, limb_name);
    encodeUInt32(payload, motor_id);
    encodeMotorValue(payload, velocity);

    std::lock_guard<std::mutex> lock(socket_mutex_);
    return sendAndWaitAck(CommandType::SET_MOTOR_VELOCITY, payload);
}

DriverError arhaTCPDriver::setEffort(const std::string& limb_name,
                                      size_t joint_index, double effort) {
    auto err = validateLimbJoint(limb_name, joint_index);
    if (err != DriverError::SUCCESS) return err;

    uint32_t motor_id = limbs_.at(limb_name).motor_ids[joint_index];

    std::vector<uint8_t> payload;
    encodeString(payload, limb_name);
    encodeUInt32(payload, motor_id);
    encodeMotorValue(payload, effort);

    std::lock_guard<std::mutex> lock(socket_mutex_);
    return sendAndWaitAck(CommandType::SET_MOTOR_EFFORT, payload);
}

DriverError arhaTCPDriver::getState(const std::string& limb_name,
                                     size_t joint_index,
                                     double& position, double& velocity,
                                     double& effort) {
    auto err = validateLimbJoint(limb_name, joint_index);
    if (err != DriverError::SUCCESS) return err;

    uint32_t motor_id = limbs_.at(limb_name).motor_ids[joint_index];

    std::vector<uint8_t> payload;
    encodeString(payload, limb_name);
    encodeUInt32(payload, motor_id);

    std::lock_guard<std::mutex> lock(socket_mutex_);
    err = sendPacket(CommandType::GET_MOTOR_STATE, payload);
    if (err != DriverError::SUCCESS) return err;

    std::vector<uint8_t> response;
    err = receivePacket(response);
    if (err != DriverError::SUCCESS) return err;

    if (response.size() < 12) {
        return setLastError(DriverError::INVALID_DATA,
            "Response too short for motor state: got " +
            std::to_string(response.size()) + ", expected 12");
    }

    position = decodeMotorValue(response, 0);
    velocity = decodeMotorValue(response, 4);
    effort   = decodeMotorValue(response, 8);

    return DriverError::SUCCESS;
}

// Emergency

DriverError arhaTCPDriver::emergencyStop() {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    std::vector<uint8_t> empty;
    return sendAndWaitAck(CommandType::EMERGENCY_STOP, empty);
}

DriverError arhaTCPDriver::emergencyStopLimb(const std::string& limb_name) {
    auto err = validateLimb(limb_name);
    if (err != DriverError::SUCCESS) return err;

    const auto& limb = limbs_.at(limb_name);
    std::vector<uint8_t> payload;
    encodeString(payload, limb_name);
    encodeUInt8(payload, static_cast<uint8_t>(limb.motor_ids.size()));
    for (auto id : limb.motor_ids) {
        encodeUInt32(payload, id);
    }

    std::lock_guard<std::mutex> lock(socket_mutex_);
    return sendAndWaitAck(CommandType::EMERGENCY_STOP_LIMB, payload);
}

DriverError arhaTCPDriver::resetErrors() {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    std::vector<uint8_t> empty;
    return sendAndWaitAck(CommandType::RESET_ERRORS, empty);
}

DriverError arhaTCPDriver::enableMotors(bool enable) {
    std::vector<uint8_t> payload;
    encodeUInt8(payload, enable ? 1 : 0);

    std::lock_guard<std::mutex> lock(socket_mutex_);
    return sendAndWaitAck(CommandType::ENABLE_MOTORS, payload);
}

DriverError arhaTCPDriver::enableLimbMotors(const std::string& limb_name, bool enable) {
    auto err = validateLimb(limb_name);
    if (err != DriverError::SUCCESS) return err;

    const auto& limb = limbs_.at(limb_name);
    std::vector<uint8_t> payload;
    encodeString(payload, limb_name);
    encodeUInt8(payload, enable ? 1 : 0);
    encodeUInt8(payload, static_cast<uint8_t>(limb.motor_ids.size()));
    for (auto id : limb.motor_ids) {
        encodeUInt32(payload, id);
    }

    std::lock_guard<std::mutex> lock(socket_mutex_);
    return sendAndWaitAck(CommandType::ENABLE_LIMB_MOTORS, payload);
}

DriverError arhaTCPDriver::setEncoderZero(const std::string& limb_name) {
    auto err = validateLimb(limb_name);
    if (err != DriverError::SUCCESS) return err;

    const auto& limb = limbs_.at(limb_name);
    std::vector<uint8_t> payload;
    encodeString(payload, limb_name);
    encodeUInt8(payload, static_cast<uint8_t>(limb.motor_ids.size()));
    for (auto id : limb.motor_ids) {
        encodeUInt32(payload, id);
    }

    std::lock_guard<std::mutex> lock(socket_mutex_);
    err = sendPacket(CommandType::SET_ENCODER_ZERO, payload);
    if (err != DriverError::SUCCESS) return err;

    auto old_timeout = config_.socket_timeout_ms;
    struct timeval tv;
    tv.tv_sec = 45;
    tv.tv_usec = 0;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    std::vector<uint8_t> response;
    err = receivePacket(response);

    tv.tv_sec = old_timeout / 1000;
    tv.tv_usec = (old_timeout % 1000) * 1000;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    if (err != DriverError::SUCCESS) return err;

    if (response.empty() || response[0] != 1) {
        std::string err_detail = response.empty() ? "empty response" :
            ("response code " + std::to_string(response[0]));
        return setLastError(DriverError::INVALID_DATA,
            "Encoder zero failed for limb '" + limb_name + "': " + err_detail);
    }
    return DriverError::SUCCESS;
}

// Accel

DriverError arhaTCPDriver::readAccel(const std::string& limb_name,
                                     std::vector<uint32_t>& accel_values) {
    auto err = validateLimb(limb_name);
    if (err != DriverError::SUCCESS) return err;

    const auto& limb = limbs_.at(limb_name);
    std::vector<uint8_t> payload;
    encodeString(payload, limb_name);
    encodeUInt8(payload, static_cast<uint8_t>(limb.motor_ids.size()));
    for (auto id : limb.motor_ids) {
        encodeUInt32(payload, id);
    }

    std::lock_guard<std::mutex> lock(socket_mutex_);
    err = sendPacket(CommandType::READ_ACCEL, payload);
    if (err != DriverError::SUCCESS) return err;

    std::vector<uint8_t> response;
    err = receivePacket(response);
    if (err != DriverError::SUCCESS) return err;

    size_t num = limb.motor_ids.size();
    if (response.size() < num * 8) {
        return setLastError(DriverError::INVALID_DATA,
            "Response too short for accel read");
    }

    accel_values.resize(num);
    for (size_t i = 0; i < num; ++i) {
        accel_values[i] = decodeUInt32(response, i * 8 + 4);
    }
    return DriverError::SUCCESS;
}

DriverError arhaTCPDriver::writeAccel(const std::string& limb_name,
                                      const std::vector<uint32_t>& accel_values) {
    auto err = validateLimb(limb_name);
    if (err != DriverError::SUCCESS) return err;

    const auto& limb = limbs_.at(limb_name);
    if (accel_values.size() != limb.motor_ids.size()) {
        return setLastError(DriverError::JOINT_COUNT_MISMATCH,
            "accel_values size mismatch");
    }

    std::vector<uint8_t> payload;
    encodeString(payload, limb_name);
    encodeUInt8(payload, static_cast<uint8_t>(limb.motor_ids.size()));
    for (size_t i = 0; i < limb.motor_ids.size(); ++i) {
        encodeUInt32(payload, limb.motor_ids[i]);
        encodeUInt32(payload, accel_values[i]);
    }

    std::lock_guard<std::mutex> lock(socket_mutex_);
    return sendAndWaitAck(CommandType::WRITE_ACCEL, payload);
}

// Gripper

DriverError arhaTCPDriver::gripperPing(uint8_t gripper_index) {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    std::vector<uint8_t> payload;
    payload.push_back(gripper_index);
    return sendAndWaitAck(CommandType::GRIPPER_PING, payload);
}

DriverError arhaTCPDriver::gripperOpen(uint8_t gripper_index, uint16_t speed) {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    std::vector<uint8_t> payload;
    payload.push_back(gripper_index);
    payload.push_back(speed & 0xFF);
    payload.push_back((speed >> 8) & 0xFF);
    return sendAndWaitAck(CommandType::GRIPPER_OPEN, payload);
}

DriverError arhaTCPDriver::gripperClose(uint8_t gripper_index, uint16_t speed) {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    std::vector<uint8_t> payload;
    payload.push_back(gripper_index);
    payload.push_back(speed & 0xFF);
    payload.push_back((speed >> 8) & 0xFF);
    return sendAndWaitAck(CommandType::GRIPPER_CLOSE, payload);
}

DriverError arhaTCPDriver::gripperMoveTo(uint8_t gripper_index,
                                          uint16_t position, uint16_t speed) {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    std::vector<uint8_t> payload;
    payload.push_back(gripper_index);
    payload.push_back(position & 0xFF);
    payload.push_back((position >> 8) & 0xFF);
    payload.push_back(speed & 0xFF);
    payload.push_back((speed >> 8) & 0xFF);
    return sendAndWaitAck(CommandType::GRIPPER_MOVE_TO, payload);
}

DriverError arhaTCPDriver::gripperGetStatus(uint8_t gripper_index,
                                             uint16_t& position, int16_t& speed,
                                             int16_t& load, uint8_t& voltage_decivolts,
                                             uint8_t& temp_c) {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    std::vector<uint8_t> payload;
    payload.push_back(gripper_index);

    auto err = sendPacket(CommandType::GRIPPER_GET_STATUS, payload);
    if (err != DriverError::SUCCESS) return err;

    std::vector<uint8_t> response;
    err = receivePacket(response);
    if (err != DriverError::SUCCESS) return err;

    if (response.empty()) {
        return setLastError(DriverError::INVALID_DATA, "Empty response from gripper");
    }

    if (response[0] != 1) {
        std::string err_type = "Unknown Error";
        if (response[0] == 2)      err_type = "UART Frame Error (Collision/Noise)";
        else if (response[0] == 3) err_type = "Motor Status Error (Protective Stop)";
        else if (response[0] == 4) err_type = "UART Timeout (Servo Offline)";
        else if (response[0] == 0) err_type = "Firmware Logic Error";

        return setLastError(DriverError::INVALID_DATA, "Gripper read failed: " + err_type);
    }

    if (response.size() < 9) {
        return setLastError(DriverError::INVALID_DATA,
            "Response too short for gripper status: got " +
            std::to_string(response.size()) + ", expected 9");
    }

    position          = response[1] | (response[2] << 8);
    speed             = (int16_t)(response[3] | (response[4] << 8));
    load              = (int16_t)(response[5] | (response[6] << 8));
    voltage_decivolts = response[7];
    temp_c            = response[8];

    return DriverError::SUCCESS;
}

DriverError arhaTCPDriver::gripperSetID(uint8_t gripper_index, uint8_t new_id) {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    std::vector<uint8_t> payload;
    payload.push_back(gripper_index);
    payload.push_back(new_id);
    return sendAndWaitAck(CommandType::GRIPPER_SET_ID, payload);
}

DriverError arhaTCPDriver::gripperForceSetID(uint8_t gripper_index, uint8_t new_id) {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    std::vector<uint8_t> payload;
    payload.push_back(gripper_index);
    payload.push_back(new_id);
    return sendAndWaitAck(CommandType::GRIPPER_SET_ID_BROADCAST, payload);
}

DriverError arhaTCPDriver::gripperScan(uint8_t gripper_index, std::vector<uint8_t>& ids_found) {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    ids_found.clear();

    std::vector<uint8_t> payload;
    payload.push_back(gripper_index);

    auto err = sendPacket(CommandType::GRIPPER_SCAN, payload);
    if (err != DriverError::SUCCESS) return err;

    std::vector<uint8_t> response;
    err = receivePacket(response);
    if (err != DriverError::SUCCESS) return err;

    if (response.size() < 2) {
        return setLastError(DriverError::INVALID_DATA, "Scan response too short");
    }

    if (response[0] != 1) {
        return setLastError(DriverError::INVALID_DATA, "Scan failed on firmware");
    }

    uint8_t count = response[1];
    if (response.size() < static_cast<size_t>(2 + count)) {
        return setLastError(DriverError::INVALID_DATA, "Scan payload mismatch");
    }

    for (uint8_t i = 0; i < count; ++i) {
        ids_found.push_back(response[2 + i]);
    }

    return DriverError::SUCCESS;
}

// Diagnostics

std::string arhaTCPDriver::getLastErrorMessage() const {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_message_;
}

void arhaTCPDriver::setErrorCallback(ErrorCallback callback) {
    std::lock_guard<std::mutex> lock(error_mutex_);
    error_callback_ = std::move(callback);
}

// Validation (private)

DriverError arhaTCPDriver::validateLimb(const std::string& limb_name) const {
    if (!connected_.load()) {
        return DriverError::NOT_CONNECTED;
    }
    if (limbs_.count(limb_name) == 0) {
        return DriverError::UNKNOWN_LIMB;
    }
    return DriverError::SUCCESS;
}

DriverError arhaTCPDriver::validateLimbJoint(const std::string& limb_name,
                                              size_t joint_index) const {
    auto err = validateLimb(limb_name);
    if (err != DriverError::SUCCESS) return err;

    if (joint_index >= limbs_.at(limb_name).motor_ids.size()) {
        return DriverError::JOINT_COUNT_MISMATCH;
    }
    return DriverError::SUCCESS;
}

// Low-level communication (private)

DriverError arhaTCPDriver::sendPacket(CommandType cmd,
                                       const std::vector<uint8_t>& data) {
    if (!connected_.load()) {
        return setLastError(DriverError::NOT_CONNECTED, "Not connected");
    }

    uint16_t payload_len = static_cast<uint16_t>(data.size());

    std::vector<uint8_t> frame;
    frame.reserve(6 + data.size());

    frame.push_back(START_BYTE);
    frame.push_back(static_cast<uint8_t>(cmd));
    frame.push_back(static_cast<uint8_t>(payload_len & 0xFF));
    frame.push_back(static_cast<uint8_t>((payload_len >> 8) & 0xFF));
    frame.insert(frame.end(), data.begin(), data.end());

    if (config_.enable_checksum) {
        uint8_t cksum = 0;
        for (size_t i = 1; i < frame.size(); ++i) {
            cksum ^= frame[i];
        }
        frame.push_back(cksum);
    }

    frame.push_back(END_BYTE);

    size_t total_sent = 0;
    while (total_sent < frame.size()) {
        ssize_t sent = ::send(socket_fd_,
                              frame.data() + total_sent,
                              frame.size() - total_sent,
                              MSG_NOSIGNAL);
        if (sent < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return setLastError(DriverError::TIMEOUT, "Send timed out");
            }
            connected_.store(false);
            return setLastError(DriverError::SEND_FAILED,
                "send() failed: " + std::string(std::strerror(errno)));
        }
        total_sent += static_cast<size_t>(sent);
    }

    return DriverError::SUCCESS;
}

DriverError arhaTCPDriver::sendAndWaitAck(CommandType cmd,
                                           const std::vector<uint8_t>& data) {
    auto err = sendPacket(cmd, data);
    if (err != DriverError::SUCCESS) return err;

    std::vector<uint8_t> ack;
    err = receivePacket(ack);
    if (err != DriverError::SUCCESS) {
        closeSocket();
        connected_.store(false);
        return setLastError(err, "ACK receive failed for cmd 0x" +
            ([](uint8_t v) {
                char buf[8];
                std::snprintf(buf, sizeof(buf), "%02X", v);
                return std::string(buf);
            })(static_cast<uint8_t>(cmd)) +
            " — socket closed, call reconnect()");
    }

    return DriverError::SUCCESS;
}

DriverError arhaTCPDriver::receivePacket(std::vector<uint8_t>& data) {
    if (!connected_.load()) {
        return setLastError(DriverError::NOT_CONNECTED, "Not connected");
    }

    data.clear();

    auto recv_exact = [this](uint8_t* buf, size_t len) -> DriverError {
        size_t total = 0;
        while (total < len) {
            ssize_t n = ::recv(socket_fd_, buf + total, len - total, 0);
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    return DriverError::TIMEOUT;
                }
                connected_.store(false);
                return DriverError::RECEIVE_FAILED;
            }
            if (n == 0) {
                connected_.store(false);
                return DriverError::RECEIVE_FAILED;
            }
            total += static_cast<size_t>(n);
        }
        return DriverError::SUCCESS;
    };

    uint8_t start = 0;
    auto err = recv_exact(&start, 1);
    if (err != DriverError::SUCCESS) return setLastError(err, "Failed to receive start byte");
    if (start != START_BYTE) {
        return setLastError(DriverError::INVALID_DATA, "Invalid start byte");
    }

    uint8_t header[3];
    err = recv_exact(header, 3);
    if (err != DriverError::SUCCESS) return setLastError(err, "Failed to receive header");

    uint16_t payload_len = static_cast<uint16_t>(header[1]) |
                           (static_cast<uint16_t>(header[2]) << 8);

    std::vector<uint8_t> payload(payload_len);
    if (payload_len > 0) {
        err = recv_exact(payload.data(), payload_len);
        if (err != DriverError::SUCCESS) return setLastError(err, "Failed to receive payload");
    }

    if (config_.enable_checksum) {
        uint8_t trailer[2];
        err = recv_exact(trailer, 2);
        if (err != DriverError::SUCCESS) return setLastError(err, "Failed to receive trailer");

        uint8_t received_cksum = trailer[0];
        uint8_t end_byte = trailer[1];

        uint8_t calc_cksum = 0;
        for (int i = 0; i < 3; ++i) calc_cksum ^= header[i];
        for (auto b : payload) calc_cksum ^= b;

        if (received_cksum != calc_cksum) {
            return setLastError(DriverError::CHECKSUM_ERROR, "Checksum mismatch");
        }
        if (end_byte != END_BYTE) {
            return setLastError(DriverError::INVALID_DATA, "Invalid end byte");
        }
    } else {
        uint8_t end_byte = 0;
        err = recv_exact(&end_byte, 1);
        if (err != DriverError::SUCCESS) return setLastError(err, "Failed to receive end byte");
        if (end_byte != END_BYTE) {
            return setLastError(DriverError::INVALID_DATA, "Invalid end byte");
        }
    }

    data = std::move(payload);
    return DriverError::SUCCESS;
}

void arhaTCPDriver::encodeUInt32(std::vector<uint8_t>& buffer, uint32_t value) {
    buffer.push_back(static_cast<uint8_t>(value & 0xFF));
    buffer.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
    buffer.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
    buffer.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
}

void arhaTCPDriver::encodeMotorValue(std::vector<uint8_t>& buffer, double radians) {
    float f = static_cast<float>(radians);
    uint8_t bytes[4];
    std::memcpy(bytes, &f, 4);
    buffer.insert(buffer.end(), bytes, bytes + 4);
}

void arhaTCPDriver::encodeString(std::vector<uint8_t>& buffer, const std::string& str) {
    encodeUInt8(buffer, static_cast<uint8_t>(str.size()));
    buffer.insert(buffer.end(), str.begin(), str.end());
}

uint32_t arhaTCPDriver::decodeUInt32(const std::vector<uint8_t>& buffer, size_t offset) {
    return static_cast<uint32_t>(buffer[offset]) |
           (static_cast<uint32_t>(buffer[offset + 1]) << 8) |
           (static_cast<uint32_t>(buffer[offset + 2]) << 16) |
           (static_cast<uint32_t>(buffer[offset + 3]) << 24);
}

int32_t arhaTCPDriver::decodeInt32(const std::vector<uint8_t>& buffer, size_t offset) {
    int32_t value;
    std::memcpy(&value, buffer.data() + offset, 4);
    return value;
}

double arhaTCPDriver::decodeMotorValue(const std::vector<uint8_t>& buffer, size_t offset) {
    float f;
    std::memcpy(&f, buffer.data() + offset, 4);
    return static_cast<double>(f);
}

DriverError arhaTCPDriver::createSocket() {
    socket_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd_ < 0) {
        return setLastError(DriverError::CONNECTION_FAILED,
            "socket() failed: " + std::string(std::strerror(errno)));
    }

    int flag = 1;
    setsockopt(socket_fd_, IPPROTO_TCP, TCP_NODELAY,
               reinterpret_cast<char*>(&flag), sizeof(flag));

    return DriverError::SUCCESS;
}

void arhaTCPDriver::closeSocket() {
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
}

DriverError arhaTCPDriver::setSocketTimeout(int timeout_ms) {
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        return setLastError(DriverError::CONNECTION_FAILED,
            "Failed to set receive timeout: " + std::string(std::strerror(errno)));
    }
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) < 0) {
        return setLastError(DriverError::CONNECTION_FAILED,
            "Failed to set send timeout: " + std::string(std::strerror(errno)));
    }
    return DriverError::SUCCESS;
}

DriverError arhaTCPDriver::setLastError(DriverError error, const std::string& message) {
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_ = error;
        last_error_message_ = message;

        if (error_callback_) {
            error_callback_(error, message);
        }
    }

    if (error != DriverError::SUCCESS) {
        logError(message);
    }
    return error;
}

void arhaTCPDriver::logError(const std::string& /*message*/) {
    // Debug output removed
}

void arhaTCPDriver::logInfo(const std::string& /*message*/) {
    // Debug output removed
}

} // namespace arha_tcp_driver