#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include <frc/SerialPort.h>

class RoboClawDriver {
 public:
  struct EncoderResult {
    int32_t count;
    uint8_t status;
  };

  explicit RoboClawDriver(frc::SerialPort::Port port, int baudRate = 38400);
  ~RoboClawDriver() = default;
  RoboClawDriver(const RoboClawDriver&) = delete;
  RoboClawDriver& operator=(const RoboClawDriver&) = delete;

  bool SetM1Speed(uint8_t address, int32_t speed);
  bool SetM2Speed(uint8_t address, int32_t speed);
  bool SetM1M2Speed(uint8_t address, int32_t speedM1, int32_t speedM2);
  bool SetM1VelocityPID(uint8_t address, float kp, float ki, float kd,
                        uint32_t qpps);
  bool SetM2VelocityPID(uint8_t address, float kp, float ki, float kd,
                        uint32_t qpps);
  bool WriteNVM(uint8_t address);

  std::optional<EncoderResult> ReadM1Encoder(uint8_t address);
  std::optional<EncoderResult> ReadM2Encoder(uint8_t address);
  bool ResetEncoders(uint8_t address);

  std::optional<std::string> ReadFirmwareVersion(uint8_t address);
  uint32_t GetErrorCount() const;
  void ResetErrorCount();

 private:
  static uint16_t CRC16(const uint8_t* data, size_t length);
  static void CRC16Update(uint16_t& crc, uint8_t byte);
  bool WriteCommand(const uint8_t* packet, size_t length);
  bool ReadCommand(const uint8_t* header, size_t headerLen, uint8_t* response,
                   size_t responseLen);
  void FlushInput();
  static void PackInt32BE(uint8_t* dest, int32_t value);
  static int32_t UnpackInt32BE(const uint8_t* src);
  static uint16_t UnpackUint16BE(const uint8_t* src);

  frc::SerialPort m_serial;
  mutable std::mutex m_mutex;
  uint32_t m_errorCount{0};

  static constexpr int kReadTimeoutMs = 3;
  static constexpr size_t kMaxResponseLen = 48;
  static constexpr uint8_t kAck = 0xFF;
  static constexpr uint8_t kCmdReadM1Encoder = 16;
  static constexpr uint8_t kCmdReadM2Encoder = 17;
  static constexpr uint8_t kCmdResetEncoders = 20;
  static constexpr uint8_t kCmdReadFirmware = 21;
  static constexpr uint8_t kCmdSetM1Speed = 35;
  static constexpr uint8_t kCmdSetM2Speed = 36;
  static constexpr uint8_t kCmdSetM1M2Speed = 37;
  static constexpr uint8_t kCmdSetM1VelPID = 28;
  static constexpr uint8_t kCmdSetM2VelPID = 29;
  static constexpr uint8_t kCmdWriteNVM = 94;
};
