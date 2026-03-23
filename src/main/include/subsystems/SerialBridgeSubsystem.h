#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <frc/SerialPort.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>

#pragma pack(push, 1)
struct RioToJetsonPacket {
  uint8_t magic[2];
  uint8_t seq;
  float odom_x;
  float odom_y;
  float odom_theta;
  float odom_vx;
  float odom_vy;
  float odom_vtheta;
  float gyro_yaw_rate;
  float accel_x;
  float accel_y;
  int32_t enc_left;
  int32_t enc_right;
  int32_t enc_horiz;
  uint8_t hall_event;
  uint32_t timestamp_ms;
  uint8_t match_state;
  uint16_t match_time_ms;
  uint8_t crc8;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct JetsonToRioPacket {
  uint8_t magic[2];
  uint8_t seq;
  float cmd_vx;
  float cmd_vy;
  float cmd_omega;
  float beacon_arm_pos;
  float container_arm_pos;
  float sort_gate_pos;
  uint8_t flags;  // Bit0: start_signal_fwd, Bit1: software_enable
  uint8_t reserved;
  uint8_t crc8;
};
#pragma pack(pop)

static_assert(sizeof(RioToJetsonPacket) == 60,
              "RioToJetsonPacket must be 60 bytes");
static_assert(sizeof(JetsonToRioPacket) == 30,
              "JetsonToRioPacket must be 30 bytes");

class SerialBridgeSubsystem : public frc2::SubsystemBase {
 public:
  explicit SerialBridgeSubsystem(
      frc::SerialPort::Port port = frc::SerialPort::Port::kUSB1);
  ~SerialBridgeSubsystem();

  void Periodic() override;

  void SetOdometry(double x, double y, double theta, double vx, double vy,
                   double vtheta);
  void SetIMUData(double gyroYawRate, double accelX = 0.0, double accelY = 0.0);
  void SetEncoders(int32_t left, int32_t right, int32_t horiz);
  void SetHallEvent(uint8_t eventCode);
  void SetMatchState(uint8_t state);
  void SetMatchTimeMs(uint16_t ms);

  double GetCmdVx() const;
  double GetCmdVy() const;
  double GetCmdOmega() const;
  float GetBeaconArmPos() const;
  float GetContainerArmPos() const;
  float GetSortGatePos() const;
  bool GetStartSignalForwarded() const;
  bool GetSoftwareEnable() const {
    return (m_rxPacket.flags & 0x02) != 0;
  }
  bool IsJetsonConnected() const;
  uint32_t GetRxCount() const;
  uint32_t GetDroppedCount() const;

 private:
  static uint8_t ComputeCRC8(const uint8_t* data, size_t length);

  static constexpr const char* JETSON_IP = "10.0.67.5";
  static constexpr int TX_PORT = 5800;
  static constexpr int RX_PORT = 5801;

  int m_txSock{-1};
  int m_rxSock{-1};
  struct sockaddr_in m_jetsonAddr{};
  bool m_socketsReady{false};
  RioToJetsonPacket m_txPacket{};
  JetsonToRioPacket m_rxPacket{};
  uint8_t m_txSeq{0};
  uint8_t m_lastRxSeq{0};
  bool m_jetsonConnected{false};
  frc::Timer m_lastRxTimer;  // resets on every valid packet; expires = disconnected
  // 200ms = 10 missed packets at 50 Hz. Adjust if Jetson send rate changes.
  static constexpr double kJetsonTimeoutSec = 0.200;
  uint32_t m_rxCount{0};
  uint32_t m_droppedCount{0};
};
