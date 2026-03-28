#include "RoboClawDriver.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <thread>

#include <units/time.h>

namespace roboclaw {
namespace {
constexpr uint8_t kAck = 0xFF;
constexpr uint8_t kCmdReadM1Encoder = 16;
constexpr uint8_t kCmdReadM2Encoder = 17;
constexpr uint8_t kCmdResetEncoders = 20;
constexpr uint8_t kCmdReadFirmware = 21;
constexpr uint8_t kCmdSetM1Speed = 35;
constexpr uint8_t kCmdSetM2Speed = 36;
constexpr uint8_t kCmdSetM1M2Speed = 37;
constexpr uint8_t kCmdSetM1VelPID = 28;
constexpr uint8_t kCmdSetM2VelPID = 29;
constexpr uint8_t kCmdReadM1VelPID = 55;
constexpr uint8_t kCmdReadM2VelPID = 56;
constexpr uint8_t kCmdWriteNVM = 94;
}  // namespace

RoboClawDriver::RoboClawDriver(frc::SerialPort::Port port, int baud) {
  try {
    m_serial = new frc::SerialPort(baud, port);
    m_serial->SetTimeout(0.1_s);
    m_serial->SetReadBufferSize(64);
  } catch (const std::exception& e) {
    m_serial = nullptr;
    std::fprintf(stderr, "[RoboClawDriver] SerialPort open failed: %s\n", e.what());
  }
}

RoboClawDriver::~RoboClawDriver() {
  delete m_serial;
  m_serial = nullptr;
}

uint16_t RoboClawDriver::CalcCRC16(const uint8_t* data, size_t len) {
  uint16_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000U) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021U);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

void RoboClawDriver::PackInt32BE(uint8_t* dest, int32_t value) {
  dest[0] = static_cast<uint8_t>((value >> 24) & 0xFF);
  dest[1] = static_cast<uint8_t>((value >> 16) & 0xFF);
  dest[2] = static_cast<uint8_t>((value >> 8) & 0xFF);
  dest[3] = static_cast<uint8_t>(value & 0xFF);
}

void RoboClawDriver::PackUint32BE(uint8_t* dest, uint32_t value) {
  dest[0] = static_cast<uint8_t>((value >> 24) & 0xFF);
  dest[1] = static_cast<uint8_t>((value >> 16) & 0xFF);
  dest[2] = static_cast<uint8_t>((value >> 8) & 0xFF);
  dest[3] = static_cast<uint8_t>(value & 0xFF);
}

int32_t RoboClawDriver::UnpackInt32BE(const uint8_t* src) {
  return (static_cast<int32_t>(src[0]) << 24) |
         (static_cast<int32_t>(src[1]) << 16) |
         (static_cast<int32_t>(src[2]) << 8) |
         static_cast<int32_t>(src[3]);
}

uint32_t RoboClawDriver::UnpackUint32BE(const uint8_t* src) {
  return (static_cast<uint32_t>(src[0]) << 24) |
         (static_cast<uint32_t>(src[1]) << 16) |
         (static_cast<uint32_t>(src[2]) << 8) |
         static_cast<uint32_t>(src[3]);
}

uint16_t RoboClawDriver::UnpackUint16BE(const uint8_t* src) {
  return static_cast<uint16_t>((static_cast<uint16_t>(src[0]) << 8) | src[1]);
}

void RoboClawDriver::FlushRx() {
  FlushRxUnlocked();
}

void RoboClawDriver::FlushRxUnlocked() {
  if (!m_serial) {
    return;
  }
  std::array<uint8_t, 64> discard{};
  while (m_serial->GetBytesReceived() > 0) {
    const int toRead = std::min<int>(m_serial->GetBytesReceived(), discard.size());
    m_serial->Read(reinterpret_cast<char*>(discard.data()), toRead);
  }
}

bool RoboClawDriver::SendPacket(uint8_t* buf, size_t payloadLen) {
  if (!m_serial) {
    return false;
  }
  const uint16_t crc = CalcCRC16(buf, payloadLen);
  buf[payloadLen] = static_cast<uint8_t>((crc >> 8) & 0xFF);
  buf[payloadLen + 1] = static_cast<uint8_t>(crc & 0xFF);
  const int written =
      m_serial->Write(reinterpret_cast<const char*>(buf), static_cast<int>(payloadLen + 2));
  return written == static_cast<int>(payloadLen + 2);
}

int RoboClawDriver::ReadBytes(uint8_t* dst, int n, int timeoutMs) {
  if (!m_serial) {
    return 0;
  }
  int gotTotal = 0;
  auto deadline = frc::Timer::GetFPGATimestamp() + units::millisecond_t(timeoutMs);
  while (gotTotal < n && frc::Timer::GetFPGATimestamp() < deadline) {
    const int avail = m_serial->GetBytesReceived();
    if (avail > 0) {
      const int toRead = std::min(avail, n - gotTotal);
      const int got = m_serial->Read(reinterpret_cast<char*>(dst + gotTotal), toRead);
      if (got > 0) {
        gotTotal += got;
      }
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  return gotTotal;
}

bool RoboClawDriver::SetVelocityPID(uint8_t address, uint8_t cmd,
                                    const VelocityPID& pid) {
  std::lock_guard<std::mutex> lock(m_serialMutex);
  FlushRxUnlocked();

  const uint32_t dFp = static_cast<uint32_t>(pid.Kd * 65536.0 + 0.5);
  const uint32_t pFp = static_cast<uint32_t>(pid.Kp * 65536.0 + 0.5);
  const uint32_t iFp = static_cast<uint32_t>(pid.Ki * 65536.0 + 0.5);

  uint8_t packet[20] = {address, cmd};
  PackUint32BE(&packet[2], dFp);
  PackUint32BE(&packet[6], pFp);
  PackUint32BE(&packet[10], iFp);
  PackUint32BE(&packet[14], pid.qpps);

  if (!SendPacket(packet, 18)) {
    frc::Wait(10_ms);
    return false;
  }

  uint8_t ack = 0;
  const int got = ReadBytes(&ack, 1, 20);
  const bool ok = got == 1 && ack == kAck;
  if (!ok) {
    frc::Wait(10_ms);
  }
  return ok;
}

bool RoboClawDriver::SetM1VelocityPID(uint8_t address, const VelocityPID& pid) {
  return SetVelocityPID(address, kCmdSetM1VelPID, pid);
}

bool RoboClawDriver::SetM2VelocityPID(uint8_t address, const VelocityPID& pid) {
  return SetVelocityPID(address, kCmdSetM2VelPID, pid);
}

auto RoboClawDriver::ReadVelocityPID(uint8_t address, uint8_t cmd)
    -> PIDReadResult {
  std::lock_guard<std::mutex> lock(m_serialMutex);
  PIDReadResult result;
  if (!m_serial) {
    result.error = "Serial port unavailable";
    return result;
  }
  FlushRxUnlocked();

  const uint8_t query[2] = {address, cmd};
  const int written = m_serial->Write(reinterpret_cast<const char*>(query), 2);
  if (written != 2) {
    result.error = "Write failed for PID read query";
    frc::Wait(10_ms);
    return result;
  }

  frc::Wait(10_ms);  // Give RoboClaw time to process query and begin responding.

  uint8_t resp[18] = {};
  const int got = ReadBytes(resp, 18, 100);  // 100ms is generous at 38400 baud.
  if (got < 18) {
    result.error = "Timeout: expected 18 bytes, got " + std::to_string(got);
    frc::Wait(10_ms);
    return result;
  }

  // CRC covers address + cmd + 16 data bytes (per RoboClaw manual).
  // This matches how ReadEncoder() computes its CRC, and how the controller
  // computes the CRC bytes it appends to the response.
  uint8_t crcBuf[18];
  crcBuf[0] = address;
  crcBuf[1] = cmd;
  std::memcpy(crcBuf + 2, resp, 16);
  const uint16_t calc = CalcCRC16(crcBuf, 18);
  const uint16_t recv = static_cast<uint16_t>((resp[16] << 8) | resp[17]);
  if (calc != recv) {
    char msg[96];
    std::snprintf(msg, sizeof(msg), "CRC mismatch: calc=0x%04X recv=0x%04X", calc, recv);
    result.error = msg;
    frc::Wait(10_ms);
    return result;
  }

  const uint32_t pRaw = UnpackUint32BE(&resp[0]);
  const uint32_t iRaw = UnpackUint32BE(&resp[4]);
  const uint32_t dRaw = UnpackUint32BE(&resp[8]);

  result.pid.Kp = static_cast<double>(pRaw) / 65536.0;
  result.pid.Ki = static_cast<double>(iRaw) / 65536.0;
  result.pid.Kd = static_cast<double>(dRaw) / 65536.0;
  result.pid.qpps = UnpackUint32BE(&resp[12]);
  result.ok = true;
  return result;
}

auto RoboClawDriver::ReadM1VelocityPID(uint8_t address) -> PIDReadResult {
  return ReadVelocityPID(address, kCmdReadM1VelPID);
}

auto RoboClawDriver::ReadM2VelocityPID(uint8_t address) -> PIDReadResult {
  return ReadVelocityPID(address, kCmdReadM2VelPID);
}

bool RoboClawDriver::WriteNVM(uint8_t address) {
  std::lock_guard<std::mutex> lock(m_serialMutex);
  FlushRxUnlocked();

  uint8_t packet[8] = {address, kCmdWriteNVM, 0xE2, 0x2E, 0xAB, 0x7A, 0, 0};
  if (!SendPacket(packet, 6)) {
    frc::Wait(10_ms);
    return false;
  }

  uint8_t ack = 0;
  const int got = ReadBytes(&ack, 1, 100);
  const bool gotAck = (got == 1 && ack == kAck);
  frc::Wait(2.0_s);
  if (!gotAck) {
    frc::Wait(10_ms);
  }
  return gotAck;
}

bool RoboClawDriver::SendSimpleWriteWithAck(const uint8_t* packet, size_t len,
                                            int ackTimeoutMs) {
  if (!m_serial) {
    return false;
  }
  FlushRxUnlocked();
  const int written = m_serial->Write(reinterpret_cast<const char*>(packet), static_cast<int>(len));
  if (written != static_cast<int>(len)) {
    frc::Wait(10_ms);
    return false;
  }
  uint8_t ack = 0;
  const int got = ReadBytes(&ack, 1, ackTimeoutMs);
  if (got != 1 || ack != kAck) {
    frc::Wait(10_ms);
    return false;
  }
  return true;
}

std::optional<RoboClawDriver::EncoderResult> RoboClawDriver::ReadEncoder(uint8_t address,
                                                                          uint8_t cmd) {
  if (!m_serial) {
    return std::nullopt;
  }
  FlushRxUnlocked();
  const uint8_t query[2] = {address, cmd};
  if (m_serial->Write(reinterpret_cast<const char*>(query), 2) != 2) {
    frc::Wait(10_ms);
    return std::nullopt;
  }

  uint8_t resp[7] = {};
  if (ReadBytes(resp, 7, 20) < 7) {
    frc::Wait(10_ms);
    return std::nullopt;
  }

  uint16_t combined = 0;
  for (uint8_t b : query) {
    combined ^= static_cast<uint16_t>(b) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      combined = (combined & 0x8000U) ? static_cast<uint16_t>((combined << 1) ^ 0x1021U)
                                      : static_cast<uint16_t>(combined << 1);
    }
  }
  for (int i = 0; i < 5; ++i) {
    combined ^= static_cast<uint16_t>(resp[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      combined = (combined & 0x8000U) ? static_cast<uint16_t>((combined << 1) ^ 0x1021U)
                                      : static_cast<uint16_t>(combined << 1);
    }
  }

  if (combined != UnpackUint16BE(&resp[5])) {
    frc::Wait(10_ms);
    return std::nullopt;
  }

  return EncoderResult{.count = UnpackInt32BE(resp), .status = resp[4]};
}

bool RoboClawDriver::SetM1Speed(uint8_t address, int32_t speed) {
  std::lock_guard<std::mutex> lock(m_serialMutex);
  uint8_t packet[8] = {address, kCmdSetM1Speed};
  PackInt32BE(&packet[2], speed);
  const uint16_t crc = CalcCRC16(packet, 6);
  packet[6] = static_cast<uint8_t>(crc >> 8);
  packet[7] = static_cast<uint8_t>(crc & 0xFF);
  return SendSimpleWriteWithAck(packet, 8, 20);
}

bool RoboClawDriver::SetM2Speed(uint8_t address, int32_t speed) {
  std::lock_guard<std::mutex> lock(m_serialMutex);
  uint8_t packet[8] = {address, kCmdSetM2Speed};
  PackInt32BE(&packet[2], speed);
  const uint16_t crc = CalcCRC16(packet, 6);
  packet[6] = static_cast<uint8_t>(crc >> 8);
  packet[7] = static_cast<uint8_t>(crc & 0xFF);
  return SendSimpleWriteWithAck(packet, 8, 20);
}

bool RoboClawDriver::SetM1M2Speed(uint8_t address, int32_t speedM1, int32_t speedM2) {
  std::lock_guard<std::mutex> lock(m_serialMutex);
  uint8_t packet[12] = {address, kCmdSetM1M2Speed};
  PackInt32BE(&packet[2], speedM1);
  PackInt32BE(&packet[6], speedM2);
  const uint16_t crc = CalcCRC16(packet, 10);
  packet[10] = static_cast<uint8_t>(crc >> 8);
  packet[11] = static_cast<uint8_t>(crc & 0xFF);
  return SendSimpleWriteWithAck(packet, 12, 20);
}

std::optional<RoboClawDriver::EncoderResult> RoboClawDriver::ReadM1Encoder(uint8_t address) {
  std::lock_guard<std::mutex> lock(m_serialMutex);
  return ReadEncoder(address, kCmdReadM1Encoder);
}

std::optional<RoboClawDriver::EncoderResult> RoboClawDriver::ReadM2Encoder(uint8_t address) {
  std::lock_guard<std::mutex> lock(m_serialMutex);
  return ReadEncoder(address, kCmdReadM2Encoder);
}

bool RoboClawDriver::ResetEncoders(uint8_t address) {
  std::lock_guard<std::mutex> lock(m_serialMutex);
  uint8_t packet[4] = {address, kCmdResetEncoders};
  const uint16_t crc = CalcCRC16(packet, 2);
  packet[2] = static_cast<uint8_t>(crc >> 8);
  packet[3] = static_cast<uint8_t>(crc & 0xFF);
  return SendSimpleWriteWithAck(packet, 4, 20);
}

std::optional<std::string> RoboClawDriver::ReadFirmwareVersion(uint8_t address) {
  std::lock_guard<std::mutex> lock(m_serialMutex);
  if (!m_serial) {
    return std::nullopt;
  }
  FlushRxUnlocked();

  const uint8_t header[2] = {address, kCmdReadFirmware};
  if (m_serial->Write(reinterpret_cast<const char*>(header), 2) != 2) {
    frc::Wait(10_ms);
    return std::nullopt;
  }

  uint8_t response[64] = {};
  int received = 0;
  auto deadline = frc::Timer::GetFPGATimestamp() + 50_ms;
  bool sawNull = false;
  while (frc::Timer::GetFPGATimestamp() < deadline && received < 64) {
    const int avail = m_serial->GetBytesReceived();
    if (avail > 0) {
      const int got = m_serial->Read(reinterpret_cast<char*>(response + received),
                                     std::min(avail, 64 - received));
      if (got > 0) {
        for (int i = 0; i < got; ++i) {
          if (response[received + i] == '\0') sawNull = true;
        }
        received += got;
        if (sawNull && received >= 3) break;
      }
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  if (!sawNull) {
    frc::Wait(10_ms);
    return std::nullopt;
  }

  int nullIndex = 0;
  while (nullIndex < received && response[nullIndex] != '\0') {
    ++nullIndex;
  }
  if (nullIndex + 3 > received) {
    frc::Wait(10_ms);
    return std::nullopt;
  }

  uint16_t crc = 0;
  for (uint8_t b : header) {
    crc ^= static_cast<uint16_t>(b) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                            : static_cast<uint16_t>(crc << 1);
    }
  }
  for (int i = 0; i <= nullIndex; ++i) {
    crc ^= static_cast<uint16_t>(response[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                            : static_cast<uint16_t>(crc << 1);
    }
  }

  const uint16_t recvCrc = UnpackUint16BE(&response[nullIndex + 1]);
  if (crc != recvCrc) {
    frc::Wait(10_ms);
    return std::nullopt;
  }

  return std::string(reinterpret_cast<char*>(response), static_cast<size_t>(nullIndex));
}

}  // namespace roboclaw
