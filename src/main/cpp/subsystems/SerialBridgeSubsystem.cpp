#include "subsystems/SerialBridgeSubsystem.h"

#include <algorithm>


#include <units/time.h>

namespace {
static constexpr uint8_t kMagicRioToJetson[2] = {0xA5, 0x5A};
static constexpr uint8_t kMagicJetsonToRio[2] = {0x5A, 0xA5};
constexpr uint8_t kStartSignalForwardedMask = 0x01;
}  // namespace

SerialBridgeSubsystem::SerialBridgeSubsystem(frc::SerialPort::Port port) {
  (void)port;
  SetName("SerialBridgeSubsystem");
  SetSubsystem("SerialBridgeSubsystem");

  // TX socket
  m_txSock = socket(AF_INET, SOCK_DGRAM, 0);
  if (m_txSock < 0) {
    printf("SerialBridge: failed to create TX socket\n");
    return;
  }

  // RX socket
  m_rxSock = socket(AF_INET, SOCK_DGRAM, 0);
  if (m_rxSock < 0) {
    printf("SerialBridge: failed to create RX socket\n");
    close(m_txSock);
    m_txSock = -1;
    return;
  }

  // SO_REUSEADDR on RX
  int opt = 1;
  setsockopt(m_rxSock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  // Non-blocking RX
  fcntl(m_rxSock, F_SETFL, O_NONBLOCK);

  // Bind RX to all interfaces on port 5801
  struct sockaddr_in rxAddr{};
  rxAddr.sin_family = AF_INET;
  rxAddr.sin_addr.s_addr = INADDR_ANY;
  rxAddr.sin_port = htons(RX_PORT);
  if (bind(m_rxSock, reinterpret_cast<struct sockaddr*>(&rxAddr), sizeof(rxAddr)) < 0) {
    printf("SerialBridge: failed to bind RX socket\n");
    close(m_txSock);
    m_txSock = -1;
    close(m_rxSock);
    m_rxSock = -1;
    return;
  }

  // Configure Jetson destination address
  m_jetsonAddr.sin_family = AF_INET;
  m_jetsonAddr.sin_port = htons(TX_PORT);
  inet_pton(AF_INET, JETSON_IP, &m_jetsonAddr.sin_addr);

  m_socketsReady = true;
  std::memset(&m_txPacket, 0, sizeof(m_txPacket));
  std::memset(&m_rxPacket, 0, sizeof(m_rxPacket));
  // Start timer immediately so it expires before the first packet arrives,
  // keeping m_jetsonConnected = false until a real packet is received.
  m_lastRxTimer.Start();
}

SerialBridgeSubsystem::~SerialBridgeSubsystem() {
  if (m_txSock >= 0) close(m_txSock);
  if (m_rxSock >= 0) close(m_rxSock);
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
  if (!m_socketsReady) {
    m_jetsonConnected = false;
    return;
  }

  // TX
  m_txPacket.magic[0] = kMagicRioToJetson[0];
  m_txPacket.magic[1] = kMagicRioToJetson[1];
  m_txPacket.seq = m_txSeq++;
  m_txPacket.timestamp_ms = static_cast<uint32_t>(
      frc::Timer::GetFPGATimestamp().value() * 1000.0);
  m_txPacket.crc8 = ComputeCRC8(
      reinterpret_cast<const uint8_t*>(&m_txPacket),
      sizeof(RioToJetsonPacket) - 1);
  sendto(m_txSock,
         reinterpret_cast<const char*>(&m_txPacket),
         sizeof(RioToJetsonPacket),
         0,
         reinterpret_cast<const struct sockaddr*>(&m_jetsonAddr),
         sizeof(m_jetsonAddr));
  // sendto failure silently ignored — Jetson unreachable is not fatal

  // RX: drain all queued datagrams and keep the most recent valid packet.
  // This avoids consuming stale commands if the Jetson briefly runs faster
  // than this loop.
  uint8_t rxBuf[sizeof(JetsonToRioPacket)];
  while (true) {
    ssize_t received = recvfrom(m_rxSock, rxBuf, sizeof(rxBuf), 0, nullptr, nullptr);
    if (received < 0) {
      break;  // non-blocking socket: no more data this cycle
    }
    if (received != static_cast<ssize_t>(sizeof(JetsonToRioPacket))) {
      continue;
    }
    if (rxBuf[0] != kMagicJetsonToRio[0] || rxBuf[1] != kMagicJetsonToRio[1]) {
      continue;
    }

    uint8_t expectedCrc = ComputeCRC8(rxBuf, sizeof(JetsonToRioPacket) - 1);
    if (rxBuf[sizeof(JetsonToRioPacket) - 1] != expectedCrc) {
      continue;
    }

    std::memcpy(&m_rxPacket, rxBuf, sizeof(JetsonToRioPacket));
    uint8_t expectedSeq = static_cast<uint8_t>(m_lastRxSeq + 1);
    if (m_rxCount > 0 && m_rxPacket.seq != expectedSeq) {
      m_droppedCount += static_cast<uint8_t>(m_rxPacket.seq - expectedSeq);
    }
    m_lastRxSeq = m_rxPacket.seq;
    m_rxCount++;
    m_lastRxTimer.Restart();
  }

  // Liveness timeout: if no valid packet for kJetsonTimeoutSec, mark disconnected.
  // This allows the 500ms drivetrain watchdog in RobotPeriodic() to fire on USB loss.
  m_jetsonConnected =
      m_rxCount > 0 && m_lastRxTimer.Get().value() <= kJetsonTimeoutSec;

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
