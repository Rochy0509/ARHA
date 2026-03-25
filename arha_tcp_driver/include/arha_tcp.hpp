#ifndef ARHA_TCP_HPP
#define ARHA_TCP_HPP

#include <string>
#include <vector>
#include <functional>
#include <mutex>
#include <atomic> 
#include <cstdint>
#include <cstring>
#include <unordered_map>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>


namespace arha_tcp_driver {

    /* Gripper constants */
    static constexpr uint8_t GRIPPER_RIGHT = 0;   /* USART2, servo ID 1 */
    static constexpr uint8_t GRIPPER_LEFT  = 1;   /* USART6, servo ID 2 */

    enum class DriverError {
        SUCCESS = 0,
        CONNECTION_FAILED,
        SEND_FAILED,
        RECEIVE_FAILED,
        TIMEOUT,
        INVALID_DATA,
        NOT_CONNECTED,
        CHECKSUM_ERROR,
        UNKNOWN_LIMB,
        JOINT_COUNT_MISMATCH
    };

    struct LimbConfig {
        std::string name;
        std::vector<uint32_t> motor_ids;
    };

    struct DriverConfig {
        std::string ip_address = "192.168.197.123";
        int port = 5000;
        int socket_timeout_ms = 1000;
        int connection_retry_delay_ms = 100;
        int max_connection_retries = 3;
        bool enable_checksum = true;
        bool verbose = false;
    };

    class arhaTCPDriver {
        public:
            explicit arhaTCPDriver(const DriverConfig& config);
            ~arhaTCPDriver();

            arhaTCPDriver(const arhaTCPDriver&) = delete;
            arhaTCPDriver& operator=(const arhaTCPDriver&) = delete;

            // Connection
            DriverError connect();
            void disconnect();
            bool isConnected() const;
            DriverError reconnect();

            // Registration
            DriverError registerLimb(const LimbConfig& limb);
            bool hasLimb(const std::string& limb_name) const;
            const LimbConfig* getLimbConfig(const std::string& limb_name) const;
            std::vector<std::string> getRegisteredLimbs() const;
            size_t getTotalJoints() const;

            // Batch commands
            DriverError setPositions(const std::string& limb_name,
                                    const std::vector<double>& positions);
            DriverError setVelocities(const std::string& limb_name,
                                    const std::vector<double>& velocities);
            DriverError setEfforts(const std::string& limb_name,
                                    const std::vector<double>& efforts);
            DriverError getStates(const std::string& limb_name,
                                std::vector<double>& positions,
                                std::vector<double>& velocities,
                                std::vector<double>& efforts);

            // Single-joint commands
            DriverError setPosition(const std::string& limb_name,
                                    size_t joint_index, double position);
            DriverError setVelocity(const std::string& limb_name,
                                    size_t joint_index, double velocity);
            DriverError setEffort(const std::string& limb_name,
                                    size_t joint_index, double effort);
            DriverError getState(const std::string& limb_name,
                                size_t joint_index,
                                double& position, double& velocity,
                                double& effort);

            // Emergency
            DriverError emergencyStop();
            DriverError emergencyStopLimb(const std::string& limb_name);
            DriverError resetErrors();
            DriverError enableMotors(bool enable);
            DriverError enableLimbMotors(const std::string& limb_name, bool enable);

            // Calibration
            DriverError setEncoderZero(const std::string& limb_name);
            DriverError readAccel(const std::string& limb_name,
                                 std::vector<uint32_t>& accel_values);
            DriverError writeAccel(const std::string& limb_name,
                                  const std::vector<uint32_t>& accel_values);

            // Gripper
            DriverError gripperPing(uint8_t gripper_index = GRIPPER_RIGHT);
            DriverError gripperOpen(uint8_t gripper_index = GRIPPER_RIGHT,
                                    uint16_t speed = 0);
            DriverError gripperClose(uint8_t gripper_index = GRIPPER_RIGHT,
                                     uint16_t speed = 0);
            DriverError gripperMoveTo(uint8_t gripper_index,
                                      uint16_t position, uint16_t speed = 0);
            DriverError gripperGetStatus(uint8_t gripper_index,
                                         uint16_t& position, int16_t& speed,
                                         int16_t& load, uint8_t& voltage_decivolts,
                                         uint8_t& temp_c);
            DriverError gripperSetID(uint8_t gripper_index, uint8_t new_id);
            DriverError gripperForceSetID(uint8_t gripper_index, uint8_t new_id);
            DriverError gripperScan(uint8_t gripper_index, std::vector<uint8_t>& ids_found);

            // Diagnostics
            std::string getLastErrorMessage() const;
            const DriverConfig& getConfig() const { return config_; }

            using ErrorCallback = std::function<void(DriverError, const std::string&)>;
            void setErrorCallback(ErrorCallback callback);

        private:
            static constexpr uint8_t START_BYTE = 0xAA;
            static constexpr uint8_t END_BYTE = 0x55;

            enum class CommandType : uint8_t {
                SET_MOTOR_POSITION  = 0x01,
                SET_MOTOR_VELOCITY  = 0x02,
                SET_MOTOR_EFFORT    = 0x03,
                GET_MOTOR_STATE     = 0x04,

                SET_LIMB_POSITIONS  = 0x14,
                SET_LIMB_VELOCITIES = 0x15,
                SET_LIMB_EFFORTS    = 0x16,
                GET_LIMB_STATES     = 0x17,

                EMERGENCY_STOP      = 0x20,
                EMERGENCY_STOP_LIMB = 0x21,
                RESET_ERRORS        = 0x22,
                ENABLE_MOTORS       = 0x23,
                ENABLE_LIMB_MOTORS  = 0x24,
                SET_ENCODER_ZERO    = 0x30,
                READ_ACCEL          = 0x31,
                WRITE_ACCEL         = 0x32,

                GRIPPER_PING        = 0x40,
                GRIPPER_OPEN        = 0x41,
                GRIPPER_CLOSE       = 0x42,
                GRIPPER_MOVE_TO     = 0x43,
                GRIPPER_GET_STATUS  = 0x44,
                GRIPPER_SET_ID       = 0x45,
                GRIPPER_SCAN         = 0x46,
                GRIPPER_SET_ID_BROADCAST = 0x47,

                PING                = 0xFF
            };

            DriverError validateLimb(const std::string& limb_name) const;
            DriverError validateLimbJoint(const std::string& limb_name,
                                        size_t joint_index) const;

            DriverError sendPacket(CommandType cmd, const std::vector<uint8_t>& data);
            DriverError receivePacket(std::vector<uint8_t>& data);
            DriverError sendAndWaitAck(CommandType cmd, const std::vector<uint8_t>& data);

            void encodeUInt8(std::vector<uint8_t>& buffer, uint8_t value);
            void encodeUInt32(std::vector<uint8_t>& buffer, uint32_t value);
            void encodeString(std::vector<uint8_t>& buffer, const std::string& str);
            uint32_t decodeUInt32(const std::vector<uint8_t>& buffer, size_t offset);
            int32_t decodeInt32(const std::vector<uint8_t>& buffer, size_t offset);
            void encodeMotorValue(std::vector<uint8_t>& buffer, double radians);
            double decodeMotorValue(const std::vector<uint8_t>& buffer, size_t offset);
            uint8_t calculateChecksum(const std::vector<uint8_t>& data);

            DriverError createSocket();
            void closeSocket();
            DriverError setSocketTimeout(int timeout_ms);

            DriverError setLastError(DriverError error, const std::string& message);
            void logError(const std::string& message);
            void logInfo(const std::string& message);

            DriverConfig config_;
            std::unordered_map<std::string, LimbConfig> limbs_;
            int socket_fd_{-1};
            std::atomic<bool> connected_{false};

            mutable std::mutex socket_mutex_;
            mutable std::mutex error_mutex_;

            DriverError last_error_{DriverError::SUCCESS};
            std::string last_error_message_;
            ErrorCallback error_callback_;
    };

} // namespace arha_tcp_driver

#endif //ARHA_TCP_HPP