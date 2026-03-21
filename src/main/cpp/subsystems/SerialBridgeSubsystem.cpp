#include "subsystems/SerialBridgeSubsystem.h"

#include <algorithm>

#include <units/time.h>

namespace {
static constexpr uint8_t kMagicRioToJetson[2] = {0xA5, 0x5A};
static constexpr uint8_t kMagicJetsonToRio[2] = {0x5A, 0xA5};
constexpr uint8_t kStartSignalForwardedMask = 0x01;
}  // namespace

SerialBridgeSubsystem::SerialBridgeSubsystem(frc::SerialPort::Port port)
    : m_serial(115200,
               port,
               8,
               frc::SerialPort::Parity::kParity_None,
               frc::SerialPort::StopBits::kStopBits_One) {
  SetName("SerialBridgeSubsystem");
  SetSubsystem("SerialBridgeSubsystem");

  m_serial.SetFlowControl(frc::SerialPort::FlowControl::kFlowControl_None);
  m_serial.SetWriteBufferMode(frc::SerialPort::WriteBufferMode::kFlushOnAccess);
  m_serial.SetReadBufferSize(static_cast<int>(kRxBufSize));
  m_serial.DisableTermination();
  m_serial.SetTimeout(units::second_t{0.005});

  std::memset(&m_txPacket, 0, sizeof(m_txPacket));
  std::memset(&m_rxPacket, 0, sizeof(m_rxPacket));
  // Start timer immediately so it expires before the first packet arrives,
  // keeping m_jetsonConnected = false until a real packet is received.
  m_lastRxTimer.Start();
}

uint8_t SerialBridgeSubsystem::ComputeCRC8(const uint8_t* data, size_t length) {
  uint8_t crc = 0x00;

  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x80U) != 0U) {
        crc = static_cast<uint8_t>((crc << 1U) ^ 0x07U);
      } else {
        crc = static_cast<uint8_t>(crc << 1U);
      }
    }
  }

  return crc;
}

void SerialBridgeSubsystem::Periodic() {
  std::memcpy(m_txPacket.magic, kMagicRioToJetson, sizeof(kMagicRioToJetson));
  m_txPacket.seq = m_txSeq++;
  m_txPacket.timestamp_ms = static_cast<uint32_t>(
      frc::Timer::GetFPGATimestamp().value() * 1000.0);
  m_txPacket.crc8 =
      ComputeCRC8(reinterpret_cast<const uint8_t*>(&m_txPacket), sizeof(m_txPacket) - 1);

  m_serial.Write(reinterpret_cast<const char*>(&m_txPacket),
                 static_cast<int>(sizeof(m_txPacket)));

  const int bytesAvailable = m_serial.GetBytesReceived();
  if (bytesAvailable > 0) {
    const size_t roomRemaining = kRxBufSize - m_rxBufLen;
    const int bytesToRead = static_cast<int>(std::min<size_t>(
        static_cast<size_t>(bytesAvailable), roomRemaining));

    if (bytesToRead > 0) {
      const int bytesRead = m_serial.Read(
          reinterpret_cast<char*>(m_rxBuf + m_rxBufLen), bytesToRead);
      if (bytesRead > 0) {
        m_rxBufLen += static_cast<size_t>(bytesRead);
      }
    }

    const int unreadBytes = bytesAvailable - bytesToRead;
    if (unreadBytes > 0) {
      char discard[kRxBufSize];
      const int discardCount = std::min(unreadBytes, static_cast<int>(kRxBufSize));
      m_serial.Read(discard, discardCount);
    }
  }

  constexpr size_t kPacketSize = sizeof(JetsonToRioPacket);
  size_t scanIndex = 0;

  while (m_rxBufLen >= kPacketSize && scanIndex + kPacketSize <= m_rxBufLen) {
    if (m_rxBuf[scanIndex] != kMagicJetsonToRio[0] ||
        m_rxBuf[scanIndex + 1] != kMagicJetsonToRio[1]) {
      ++scanIndex;
      continue;
    }

    JetsonToRioPacket candidate{};
    std::memcpy(&candidate, m_rxBuf + scanIndex, kPacketSize);

    const uint8_t crc =
        ComputeCRC8(reinterpret_cast<const uint8_t*>(&candidate), kPacketSize - 1);
    if (crc != candidate.crc8) {
      ++scanIndex;
      continue;
    }

    m_rxPacket = candidate;
    if (m_rxCount > 0) {
      const uint8_t expectedSeq = static_cast<uint8_t>(m_lastRxSeq + 1U);
      if (m_rxPacket.seq != expectedSeq) {
        m_droppedCount +=
            static_cast<uint8_t>(m_rxPacket.seq - expectedSeq);
      }
    }
    m_lastRxSeq = m_rxPacket.seq;
    ++m_rxCount;
    m_jetsonConnected = true;
    m_lastRxTimer.Reset();  // reset watchdog; if no packet arrives within
                            // kJetsonTimeoutSec, liveness check below clears the flag

    const size_t consumed = scanIndex + kPacketSize;
    const size_t remaining = m_rxBufLen - consumed;
    if (remaining > 0) {
      std::memmove(m_rxBuf, m_rxBuf + consumed, remaining);
    }
    m_rxBufLen = remaining;
    scanIndex = 0;
  }

  if (scanIndex > 0 && scanIndex < m_rxBufLen) {
    const size_t remaining = m_rxBufLen - scanIndex;
    std::memmove(m_rxBuf, m_rxBuf + scanIndex, remaining);
    m_rxBufLen = remaining;
  } else if (scanIndex >= m_rxBufLen) {
    m_rxBufLen = 0;
  }

  // Liveness timeout: if no valid packet for kJetsonTimeoutSec, mark disconnected.
  // This allows the 500ms drivetrain watchdog in RobotPeriodic() to fire on USB loss.
  if (m_jetsonConnected && m_lastRxTimer.Get().value() > kJetsonTimeoutSec) {
    m_jetsonConnected = false;
  }

  frc::SmartDashboard::PutBoolean("Bridge/JetsonConnected", m_jetsonConnected);
  frc::SmartDashboard::PutNumber("Bridge/TxSeq", m_txSeq);
  frc::SmartDashboard::PutNumber("Bridge/RxCount", m_rxCount);
  frc::SmartDashboard::PutNumber("Bridge/DroppedPkts", m_droppedCount);
  // Commands received from Jetson
  frc::SmartDashboard::PutNumber("Bridge/Cmd_Vx", m_rxPacket.cmd_vx);
  frc::SmartDashboard::PutNumber("Bridge/Cmd_Vy", m_rxPacket.cmd_vy);
  frc::SmartDashboard::PutNumber("Bridge/Cmd_Omega", m_rxPacket.cmd_omega);
  // Servo positions commanded by Jetson
  frc::SmartDashboard::PutNumber("Bridge/BeaconArmPos", m_rxPacket.beacon_arm_pos);
  frc::SmartDashboard::PutNumber("Bridge/ContainerArmPos", m_rxPacket.container_arm_pos);
  frc::SmartDashboard::PutNumber("Bridge/SortGatePos", m_rxPacket.sort_gate_pos);
  // Flags
  frc::SmartDashboard::PutBoolean("Bridge/StartSignalFwd",
      (m_rxPacket.flags & 0x01U) != 0U);
}

void SerialBridgeSubsystem::SetOdometry(double x,
                                        double y,
                                        double theta,
                                        double vx,
                                        double vy,
                                        double vtheta) {
  m_txPacket.odom_x = static_cast<float>(x);
  m_txPacket.odom_y = static_cast<float>(y);
  m_txPacket.odom_theta = static_cast<float>(theta);
  m_txPacket.odom_vx = static_cast<float>(vx);
  m_txPacket.odom_vy = static_cast<float>(vy);
  m_txPacket.odom_vtheta = static_cast<float>(vtheta);
}

void SerialBridgeSubsystem::SetIMUData(double gyroYawRate,
                                       double accelX,
                                       double accelY) {
  m_txPacket.gyro_yaw_rate = static_cast<float>(gyroYawRate);
  m_txPacket.accel_x = static_cast<float>(accelX);
  m_txPacket.accel_y = static_cast<float>(accelY);
}

void SerialBridgeSubsystem::SetEncoders(int32_t left, int32_t right, int32_t horiz) {
  m_txPacket.enc_left = left;
  m_txPacket.enc_right = right;
  m_txPacket.enc_horiz = horiz;
}

void SerialBridgeSubsystem::SetHallEvent(uint8_t eventCode) {
  m_txPacket.hall_event = eventCode;
}

void SerialBridgeSubsystem::SetMatchState(uint8_t state) {
  m_txPacket.match_state = state;
}

void SerialBridgeSubsystem::SetMatchTimeMs(uint16_t ms) {
  m_txPacket.match_time_ms = ms;
}

double SerialBridgeSubsystem::GetCmdVx() const {
  return m_rxPacket.cmd_vx;
}

double SerialBridgeSubsystem::GetCmdVy() const {
  return m_rxPacket.cmd_vy;
}

double SerialBridgeSubsystem::GetCmdOmega() const {
  return m_rxPacket.cmd_omega;
}

float SerialBridgeSubsystem::GetBeaconArmPos() const {
  return m_rxPacket.beacon_arm_pos;
}

float SerialBridgeSubsystem::GetContainerArmPos() const {
  return m_rxPacket.container_arm_pos;
}

float SerialBridgeSubsystem::GetSortGatePos() const {
  return m_rxPacket.sort_gate_pos;
}

bool SerialBridgeSubsystem::GetStartSignalForwarded() const {
  return (m_rxPacket.flags & kStartSignalForwardedMask) != 0U;
}

bool SerialBridgeSubsystem::IsJetsonConnected() const {
  return m_jetsonConnected;
}

uint32_t SerialBridgeSubsystem::GetRxCount() const {
  return m_rxCount;
}

uint32_t SerialBridgeSubsystem::GetDroppedCount() const {
  return m_droppedCount;
}
