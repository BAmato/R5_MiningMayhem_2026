#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include <frc/SerialPort.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>

#include "RoboClawDriver.h"

class Drivetrain : public frc2::SubsystemBase {
 public:
  struct VelocityPidSetting {
    double kp;
    double ki;
    double kd;
    uint32_t qpps;
  };

  static constexpr uint8_t kAddrVertical = 0x80;
  static constexpr uint8_t kAddrHorizontal = 0x81;
  static constexpr double kWheelBaseM = 0.254;               // CALIBRATE: measure center-to-center
  static constexpr double kWheelDiameterM = 0.072;
  static constexpr double kVertCountsPerRev = 751.8 * 4.0;   // CALIBRATE: verify with encoder test
  static constexpr double kHorizCountsPerRev = 384.5 * 4.0;  // CALIBRATE
  static constexpr double kHeadingHoldKP = 0.5;              // CALIBRATE
  static constexpr double kOmegaDeadband = 0.01;
  static constexpr double kMaxHeadingCorrection = 0.5;
  static constexpr double kLoopPeriodSec = 0.02;
  static constexpr double kPi = 3.14159265358979323846;
  static constexpr double kVertCountsPerM =
      kVertCountsPerRev / (kPi * kWheelDiameterM);
  static constexpr double kHorizCountsPerM =
      kHorizCountsPerRev / (kPi * kWheelDiameterM);

  Drivetrain();

  void Periodic() override;
  void SimulationPeriodic() override;

  void ResetDriveEncoders();
  double GetAverageDistanceMeters() const;
  void DriveArcade(double xSpeed, double zRotation);
  void StopDrive();
  void SetDriveOutputsEnabled(bool enabled);
  bool GetDriveOutputsEnabled() const;

  void Drive(double vx, double vy, double omega);
  void GetOdometry(double& x, double& y, double& theta) const;
  void ResetOdometry(double x = 0.0, double y = 0.0, double theta = 0.0);
  void SetGyroYawRate(double yawRateRadPerSec);
  double GetGyroYawRate() const;
  int32_t GetEncoderLeft() const;
  int32_t GetEncoderRight() const;
  int32_t GetEncoderHoriz() const;
  double GetHeading() const;
  bool VerifyControllers();
  std::optional<std::string> GetFirmwareVersionVertical();
  std::optional<std::string> GetFirmwareVersionHorizontal();
  RoboClawDriver* GetRoboClawDriver() { return &m_roboclaw; }

 private:
  static double Clamp(double value, double minValue, double maxValue);
  static double WrapAngleRadians(double angleRad);
  static int32_t MetersPerSecondToCountsPerSecond(double speedMetersPerSecond,
                                                  double countsPerMeter);
  static bool ConsumeDashboardButtonEdge(const char* key, bool& previousState);
  static bool PidCloseEnough(double lhs, double rhs);
  static bool PidMatches(const VelocityPidSetting& a, const VelocityPidSetting& b);

  void InitializePidDashboard();
  void PublishPidConfigValues() const;
  void PublishPidPendingValues() const;
  void PublishPidActualValues() const;
  void ReadPidPendingValuesFromDashboard();
  void LoadPidPendingFromConfig();
  void LoadPidPendingFromActual();
  bool RefreshPidActualFromControllers();
  bool ApplyPidPendingToControllers(bool writeNvm);
  void HandlePidDashboardActions();
  void UpdatePidMatchAndDirtyFlags();
  void SetPidStatus(const std::string& status, const std::string& lastError,
                    bool lastReadOk, bool lastApplyOk);

  RoboClawDriver m_roboclaw{frc::SerialPort::Port::kMXP, 38400};

  double m_cmdVx{0.0};
  double m_cmdVy{0.0};
  double m_cmdOmega{0.0};
  double m_gyroYawRateRadPerSec{0.0};

  double m_odomX{0.0};
  double m_odomY{0.0};
  double m_odomTheta{0.0};
  double m_headingHoldTarget{0.0};

  int32_t m_leftEncoderCount{0};
  int32_t m_rightEncoderCount{0};
  int32_t m_horizEncoderCount{0};
  int32_t m_prevLeftEncoderCount{0};
  int32_t m_prevRightEncoderCount{0};
  int32_t m_prevHorizEncoderCount{0};
  int m_encoderReadPhase{0};
  bool m_haveEncoderReference{false};
  bool m_driveOutputsEnabled{false};
  bool m_headingHoldActive{false};

  std::shared_ptr<nt::NetworkTable> m_driveBalanceTuningTable;
  // NT entries for Glass-tunable encoder balance parameters
  nt::NetworkTableEntry ntEncBalanceKp;
  nt::NetworkTableEntry ntDriveSpeedFraction;
  nt::NetworkTableEntry ntQPPS_M1;
  nt::NetworkTableEntry ntQPPS_M2;

  // Live values read from NT each loop
  double m_encBalanceKp{0.05};
  double m_driveSpeedFraction{0.05};
  double m_qpps_M1{1650.0};
  double m_qpps_M2{1320.0};

  VelocityPidSetting m_verticalM1Config{};
  VelocityPidSetting m_verticalM2Config{};
  VelocityPidSetting m_horizontalM1Config{};
  VelocityPidSetting m_verticalM1Pending{};
  VelocityPidSetting m_verticalM2Pending{};
  VelocityPidSetting m_horizontalM1Pending{};
  VelocityPidSetting m_verticalM1Actual{};
  VelocityPidSetting m_verticalM2Actual{};
  VelocityPidSetting m_horizontalM1Actual{};
  bool m_hasActualPidValues{false};
  bool m_autoRefreshOnBoot{true};
  bool m_pidLastReadOk{false};
  bool m_pidLastApplyOk{false};
  bool m_prevRefreshActual{false};
  bool m_prevLoadPendingFromConfig{false};
  bool m_prevLoadPendingFromActual{false};
  bool m_prevApplyPending{false};
  bool m_prevWriteNvm{false};
  bool m_prevApplyAndWriteNvm{false};
  bool m_prevFullReset{false};
};
