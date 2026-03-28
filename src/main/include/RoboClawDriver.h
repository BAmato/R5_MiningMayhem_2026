#pragma once

#include <frc/SerialPort.h>
#include <frc/Timer.h>

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

namespace roboclaw {

struct VelocityPID {
  double Kp = 1.0;
  double Ki = 0.5;
  double Kd = 0.25;
  uint32_t qpps = 44000;
};

struct PIDReadResult {
  bool ok = false;
  VelocityPID pid;
  std::string error;
};

class RoboClawDriver {
 public:
  struct EncoderResult {
    int32_t count;
    uint8_t status;
  };

  explicit RoboClawDriver(frc::SerialPort::Port port = frc::SerialPort::kMXP,
                          int baud = 38400);
  ~RoboClawDriver();

  bool SetM1VelocityPID(uint8_t address, const VelocityPID& pid);
  bool SetM2VelocityPID(uint8_t address, const VelocityPID& pid);

  PIDReadResult ReadM1VelocityPID(uint8_t address);
  PIDReadResult ReadM2VelocityPID(uint8_t address);

  bool WriteNVM(uint8_t address);

  static uint16_t CalcCRC16(const uint8_t* data, size_t len);
  void FlushRx();

  // Legacy APIs retained for existing drivetrain behavior.
  bool SetM1Speed(uint8_t address, int32_t speed);
  bool SetM2Speed(uint8_t address, int32_t speed);
  bool SetM1M2Speed(uint8_t address, int32_t speedM1, int32_t speedM2);
  std::optional<EncoderResult> ReadM1Encoder(uint8_t address);
  std::optional<EncoderResult> ReadM2Encoder(uint8_t address);
  bool ResetEncoders(uint8_t address);
  std::optional<std::string> ReadFirmwareVersion(uint8_t address);

 private:
  frc::SerialPort* m_serial = nullptr;
  mutable std::mutex m_serialMutex;
  void FlushRxUnlocked();

  bool SendPacket(uint8_t* buf, size_t payloadLen);
  int ReadBytes(uint8_t* dst, int n, int timeoutMs = 20);

  bool SetVelocityPID(uint8_t address, uint8_t cmd, const VelocityPID& pid);
  PIDReadResult ReadVelocityPID(uint8_t address, uint8_t cmd);

  bool SendSimpleWriteWithAck(const uint8_t* packet, size_t len, int ackTimeoutMs = 20);
  std::optional<EncoderResult> ReadEncoder(uint8_t address, uint8_t cmd);

  static void PackInt32BE(uint8_t* dest, int32_t value);
  static void PackUint32BE(uint8_t* dest, uint32_t value);
  static int32_t UnpackInt32BE(const uint8_t* src);
  static uint32_t UnpackUint32BE(const uint8_t* src);
  static uint16_t UnpackUint16BE(const uint8_t* src);
};

}  // namespace roboclaw

using RoboClawDriver = roboclaw::RoboClawDriver;
