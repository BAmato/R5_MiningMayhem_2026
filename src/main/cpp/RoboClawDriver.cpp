#include "RoboClawDriver.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <thread>

#include <frc/Timer.h>
#include <units/time.h>

RoboClawDriver::RoboClawDriver(frc::SerialPort::Port port, int baudRate)
    : m_serial(baudRate, port, 8,
               frc::SerialPort::Parity::kParity_None,
               frc::SerialPort::StopBits::kStopBits_One) {
  std::scoped_lock lock(m_mutex);
  m_serial.SetFlowControl(frc::SerialPort::FlowControl::kFlowControl_None);
  m_serial.SetWriteBufferMode(
      frc::SerialPort::WriteBufferMode::kFlushOnAccess);
  m_serial.SetReadBufferSize(static_cast<int>(kMaxResponseLen));
  m_serial.DisableTermination();
  m_serial.SetTimeout(units::second_t{kReadTimeoutMs / 1000.0});
  FlushInput();
}

bool RoboClawDriver::SetM1Speed(uint8_t address, int32_t speed) {
  std::scoped_lock lock(m_mutex);

  uint8_t packet[8] = {address, kCmdSetM1Speed};
  PackInt32BE(&packet[2], speed);
  const uint16_t crc = CRC16(packet, 6);
  packet[6] = static_cast<uint8_t>(crc >> 8);
  packet[7] = static_cast<uint8_t>(crc & 0xFF);
  return WriteCommand(packet, std::size(packet));
}

bool RoboClawDriver::SetM2Speed(uint8_t address, int32_t speed) {
  std::scoped_lock lock(m_mutex);

  uint8_t packet[8] = {address, kCmdSetM2Speed};
  PackInt32BE(&packet[2], speed);
  const uint16_t crc = CRC16(packet, 6);
  packet[6] = static_cast<uint8_t>(crc >> 8);
  packet[7] = static_cast<uint8_t>(crc & 0xFF);
  return WriteCommand(packet, std::size(packet));
}

bool RoboClawDriver::SetM1M2Speed(uint8_t address, int32_t speedM1,
                                  int32_t speedM2) {
  std::scoped_lock lock(m_mutex);

  uint8_t packet[12] = {address, kCmdSetM1M2Speed};
  PackInt32BE(&packet[2], speedM1);
  PackInt32BE(&packet[6], speedM2);
  const uint16_t crc = CRC16(packet, 10);
  packet[10] = static_cast<uint8_t>(crc >> 8);
  packet[11] = static_cast<uint8_t>(crc & 0xFF);
  return WriteCommand(packet, std::size(packet));
}

std::optional<RoboClawDriver::EncoderResult> RoboClawDriver::ReadM1Encoder(
    uint8_t address) {
  std::scoped_lock lock(m_mutex);

  const uint8_t header[2] = {address, kCmdReadM1Encoder};
  uint8_t response[7] = {};
  if (!ReadCommand(header, std::size(header), response, std::size(response))) {
    return std::nullopt;
  }

  return EncoderResult{.count = UnpackInt32BE(response), .status = response[4]};
}

std::optional<RoboClawDriver::EncoderResult> RoboClawDriver::ReadM2Encoder(
    uint8_t address) {
  std::scoped_lock lock(m_mutex);

  const uint8_t header[2] = {address, kCmdReadM2Encoder};
  uint8_t response[7] = {};
  if (!ReadCommand(header, std::size(header), response, std::size(response))) {
    return std::nullopt;
  }

  return EncoderResult{.count = UnpackInt32BE(response), .status = response[4]};
}

bool RoboClawDriver::ResetEncoders(uint8_t address) {
  std::scoped_lock lock(m_mutex);

  uint8_t packet[4] = {address, kCmdResetEncoders};
  const uint16_t crc = CRC16(packet, 2);
  packet[2] = static_cast<uint8_t>(crc >> 8);
  packet[3] = static_cast<uint8_t>(crc & 0xFF);
  return WriteCommand(packet, std::size(packet));
}

std::optional<std::string> RoboClawDriver::ReadFirmwareVersion(uint8_t address) {
  std::scoped_lock lock(m_mutex);

  const uint8_t header[2] = {address, kCmdReadFirmware};
  FlushInput();

  const int written = m_serial.Write(reinterpret_cast<const char*>(header),
                                     static_cast<int>(std::size(header)));
  if (written != static_cast<int>(std::size(header))) {
    ++m_errorCount;
    return std::nullopt;
  }

  uint8_t response[kMaxResponseLen] = {};
  size_t received = 0;
  bool sawNull = false;
  const auto startTime = frc::Timer::GetFPGATimestamp();
  const auto deadline = startTime + units::millisecond_t{kReadTimeoutMs};

  while (frc::Timer::GetFPGATimestamp() < deadline && received < kMaxResponseLen) {
    const int available = m_serial.GetBytesReceived();
    if (available > 0) {
      const size_t remaining = kMaxResponseLen - received;
      const int toRead = std::min<int>(available, static_cast<int>(remaining));
      const int got = m_serial.Read(
          reinterpret_cast<char*>(response + received), toRead);
      if (got > 0) {
        const size_t startIndex = received;
        received += static_cast<size_t>(got);
        for (size_t i = startIndex; i < received; ++i) {
          if (response[i] == '\0') {
            sawNull = true;
          }
        }
        if (sawNull) {
          size_t currentNullIndex = 0;
          while (currentNullIndex < received && response[currentNullIndex] != '\0') {
            ++currentNullIndex;
          }
          if (currentNullIndex < received && received >= currentNullIndex + 3) {
            break;
          }
        }
        if (received >= kMaxResponseLen) {
          break;
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  if (!sawNull || received < 3) {
    ++m_errorCount;
    return std::nullopt;
  }

  size_t nullIndex = 0;
  while (nullIndex < received && response[nullIndex] != '\0') {
    ++nullIndex;
  }

  if (nullIndex + 3 > received) {
    ++m_errorCount;
    return std::nullopt;
  }

  uint16_t crc = 0;
  for (uint8_t byte : header) {
    CRC16Update(crc, byte);
  }
  for (size_t i = 0; i <= nullIndex; ++i) {
    CRC16Update(crc, response[i]);
  }

  const uint16_t receivedCrc =
      UnpackUint16BE(&response[nullIndex + 1]);
  if (crc != receivedCrc) {
    ++m_errorCount;
    return std::nullopt;
  }

  return std::string(reinterpret_cast<const char*>(response), nullIndex);
}

uint32_t RoboClawDriver::GetErrorCount() const {
  std::scoped_lock lock(m_mutex);
  return m_errorCount;
}

void RoboClawDriver::ResetErrorCount() {
  std::scoped_lock lock(m_mutex);
  m_errorCount = 0;
}

uint16_t RoboClawDriver::CRC16(const uint8_t* data, size_t length) {
  uint16_t crc = 0;
  for (size_t i = 0; i < length; ++i) {
    CRC16Update(crc, data[i]);
  }
  return crc;
}

void RoboClawDriver::CRC16Update(uint16_t& crc, uint8_t byte) {
  crc ^= static_cast<uint16_t>(byte) << 8;
  for (int bit = 0; bit < 8; ++bit) {
    if ((crc & 0x8000U) != 0U) {
      crc = static_cast<uint16_t>((crc << 1) ^ 0x1021U);
    } else {
      crc = static_cast<uint16_t>(crc << 1);
    }
  }
}

bool RoboClawDriver::WriteCommand(const uint8_t* packet, size_t length) {
  FlushInput();

  const int written =
      m_serial.Write(reinterpret_cast<const char*>(packet), static_cast<int>(length));
  if (written != static_cast<int>(length)) {
    ++m_errorCount;
    return false;
  }

  return true;
}

bool RoboClawDriver::ReadCommand(const uint8_t* header, size_t headerLen,
                                 uint8_t* response, size_t responseLen) {
  FlushInput();

  uint8_t request[kMaxResponseLen] = {};
  if (headerLen + 2 > std::size(request)) {
    ++m_errorCount;
    return false;
  }

  std::memcpy(request, header, headerLen);
  const uint16_t requestCrc = CRC16(header, headerLen);
  request[headerLen] = static_cast<uint8_t>(requestCrc >> 8);
  request[headerLen + 1] = static_cast<uint8_t>(requestCrc & 0xFF);

  const size_t requestLen = headerLen + 2;
  const int written = m_serial.Write(reinterpret_cast<const char*>(request),
                                     static_cast<int>(requestLen));
  if (written != static_cast<int>(requestLen)) {
    ++m_errorCount;
    return false;
  }

  size_t received = 0;
  const auto startTime = frc::Timer::GetFPGATimestamp();
  const auto deadline = startTime + units::millisecond_t{kReadTimeoutMs};
  while (frc::Timer::GetFPGATimestamp() < deadline && received < responseLen) {
    const int available = m_serial.GetBytesReceived();
    if (available > 0) {
      const size_t remaining = responseLen - received;
      const int toRead = std::min<int>(available, static_cast<int>(remaining));
      const int got = m_serial.Read(
          reinterpret_cast<char*>(response + received), toRead);
      if (got > 0) {
        received += static_cast<size_t>(got);
      }
    }
    if (received < responseLen) {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  }

  if (received != responseLen || responseLen < 2) {
    ++m_errorCount;
    return false;
  }

  uint16_t crc = 0;
  for (size_t i = 0; i < headerLen; ++i) {
    CRC16Update(crc, header[i]);
  }
  for (size_t i = 0; i < responseLen - 2; ++i) {
    CRC16Update(crc, response[i]);
  }

  const uint16_t receivedCrc = UnpackUint16BE(&response[responseLen - 2]);
  if (crc != receivedCrc) {
    ++m_errorCount;
    return false;
  }

  return true;
}

void RoboClawDriver::FlushInput() {
  uint8_t discard[kMaxResponseLen] = {};
  while (m_serial.GetBytesReceived() > 0) {
    const int toRead = std::min<int>(m_serial.GetBytesReceived(),
                                     static_cast<int>(std::size(discard)));
    const int got =
        m_serial.Read(reinterpret_cast<char*>(discard), toRead);
    if (got <= 0) {
      break;
    }
  }
}

void RoboClawDriver::PackInt32BE(uint8_t* dest, int32_t value) {
  const uint32_t uValue = static_cast<uint32_t>(value);
  dest[0] = static_cast<uint8_t>((uValue >> 24) & 0xFF);
  dest[1] = static_cast<uint8_t>((uValue >> 16) & 0xFF);
  dest[2] = static_cast<uint8_t>((uValue >> 8) & 0xFF);
  dest[3] = static_cast<uint8_t>(uValue & 0xFF);
}

int32_t RoboClawDriver::UnpackInt32BE(const uint8_t* src) {
  const uint32_t value = (static_cast<uint32_t>(src[0]) << 24) |
                         (static_cast<uint32_t>(src[1]) << 16) |
                         (static_cast<uint32_t>(src[2]) << 8) |
                         static_cast<uint32_t>(src[3]);
  return static_cast<int32_t>(value);
}

uint16_t RoboClawDriver::UnpackUint16BE(const uint8_t* src) {
  return static_cast<uint16_t>((static_cast<uint16_t>(src[0]) << 8) |
                               static_cast<uint16_t>(src[1]));
}
