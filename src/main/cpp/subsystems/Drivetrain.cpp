#include "subsystems/Drivetrain.h"

#include <cmath>

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>

namespace {
constexpr Drivetrain::VelocityPidSetting kVerticalM1DefaultPid{
    .kp = 29.20736, .ki = 0.92726, .kd = 0.0, .qpps = 1650};
constexpr Drivetrain::VelocityPidSetting kVerticalM2DefaultPid{
    .kp = 33.88054, .ki = 1.12312, .kd = 0.0, .qpps = 1320};
constexpr Drivetrain::VelocityPidSetting kHorizontalM1DefaultPid{
    .kp = 20.0, .ki = 0.8, .kd = 0.0, .qpps = 2600};
#if 0
// DISABLED: replaced by encoder balance loop — re-enable by removing #if 0
// Feed-forward trim: left motor (M1) still runs ~4.2% faster than right in
// encoder measurements, so reduce commanded M1 speed to keep straight tracking.
constexpr double kLeftMotorTrim = 0.958;
#endif
static int32_t encLeftPrev = 0;
static int32_t encRightPrev = 0;
constexpr double kPidMatchTolerance = 1e-4;
constexpr const char* kPidController80M1 = "RoboClawPID/Controller_0x80/M1";
constexpr const char* kPidController80M2 = "RoboClawPID/Controller_0x80/M2";
constexpr const char* kPidController81M1 = "RoboClawPID/Controller_0x81/M1";
}  // namespace

Drivetrain::Drivetrain() {
  SetName("Drivetrain");
  SetSubsystem("Drivetrain");

  m_verticalM1Config = kVerticalM1DefaultPid;
  m_verticalM2Config = kVerticalM2DefaultPid;
  m_horizontalM1Config = kHorizontalM1DefaultPid;
  m_driveBalanceTuningTable =
      nt::NetworkTableInstance::GetDefault().GetTable("DriveBalanceTuning");
  ntEncBalanceKp = m_driveBalanceTuningTable->GetEntry("EncBalanceKp");
  ntDriveSpeedFraction = m_driveBalanceTuningTable->GetEntry("DriveSpeedFraction");
  ntQPPS_M1 = m_driveBalanceTuningTable->GetEntry("QPPS_M1");
  ntQPPS_M2 = m_driveBalanceTuningTable->GetEntry("QPPS_M2");
  ntEncBalanceKp.SetDefaultDouble(0.05);
  ntDriveSpeedFraction.SetDefaultDouble(0.05);
  ntQPPS_M1.SetDefaultDouble(1650.0);
  ntQPPS_M2.SetDefaultDouble(1320.0);

  InitializePidDashboard();
  PublishPidConfigValues();
  LoadPidPendingFromConfig();       // Pending = Config
  PublishPidPendingValues();
  PublishPidActualValues();
  UpdatePidMatchAndDirtyFlags();

  // Apply config PID values to controller RAM at every boot.
  // This ensures consistent velocity control regardless of what is in NVM.
  // No NVM write is performed — values only live in RAM until next power cycle.
  const bool applyOk =
      m_roboclaw.SetM1VelocityPID(
          kAddrVertical,
          roboclaw::VelocityPID{m_verticalM1Config.kp, m_verticalM1Config.ki,
                                m_verticalM1Config.kd, m_verticalM1Config.qpps}) &&
      m_roboclaw.SetM2VelocityPID(
          kAddrVertical,
          roboclaw::VelocityPID{m_verticalM2Config.kp, m_verticalM2Config.ki,
                                m_verticalM2Config.kd, m_verticalM2Config.qpps}) &&
      m_roboclaw.SetM1VelocityPID(
          kAddrHorizontal,
          roboclaw::VelocityPID{m_horizontalM1Config.kp, m_horizontalM1Config.ki,
                                m_horizontalM1Config.kd, m_horizontalM1Config.qpps});

  // Read back what the controllers now have, to confirm and populate Actual/*.
  if (applyOk && RefreshPidActualFromControllers()) {
    SetPidStatus("Ready", "", true, true);
  } else if (applyOk) {
    SetPidStatus("Applied; readback failed", "RAM write OK but readback failed",
                 false, true);
  } else {
    SetPidStatus("Boot apply failed", "Failed to write PID to controller RAM",
                 false, false);
  }

  ResetDriveEncoders();
}

void Drivetrain::Periodic() {
  static int dashboardCounter = 0;
  HandlePidDashboardActions();
  if (ConsumeDashboardButtonEdge("Drive/FullReset", m_prevFullReset)) {
    ResetDriveEncoders();
    ResetOdometry(0.0, 0.0, 0.0);
  }
  ReadPidPendingValuesFromDashboard();
  UpdatePidMatchAndDirtyFlags();
  // Pull latest values from Glass each cycle — allows live tuning without recompile
  m_encBalanceKp = ntEncBalanceKp.GetDouble(0.05);
  m_driveSpeedFraction = ntDriveSpeedFraction.GetDouble(0.05);
  m_qpps_M1 = ntQPPS_M1.GetDouble(1650.0);
  m_qpps_M2 = ntQPPS_M2.GetDouble(1320.0);

  m_encoderReadPhase = (m_encoderReadPhase + 1) % 3;
  if (m_encoderReadPhase == 0) {
    auto encL = m_roboclaw.ReadM1Encoder(kAddrVertical);
    if (encL) m_leftEncoderCount = encL->count;
  } else if (m_encoderReadPhase == 1) {
    auto encR = m_roboclaw.ReadM2Encoder(kAddrVertical);
    if (encR) m_rightEncoderCount = encR->count;
  } else {
    auto encH = m_roboclaw.ReadM1Encoder(kAddrHorizontal);
    if (encH) m_horizEncoderCount = encH->count;
  }

  if (!m_haveEncoderReference) {
    m_prevLeftEncoderCount = m_leftEncoderCount;
    m_prevRightEncoderCount = m_rightEncoderCount;
    m_prevHorizEncoderCount = m_horizEncoderCount;
    m_haveEncoderReference = true;
  }

  const int32_t dLeftCounts = m_leftEncoderCount - m_prevLeftEncoderCount;
  int32_t dRightCounts = m_rightEncoderCount - m_prevRightEncoderCount;
  if (m_cmdVx < 0.0) {
    // M2 encoder counts inverted in reverse — negate to match M1 sign convention.
    dRightCounts = -dRightCounts;
  }
  const int32_t dHorizCounts = m_horizEncoderCount - m_prevHorizEncoderCount;

  m_prevLeftEncoderCount = m_leftEncoderCount;
  m_prevRightEncoderCount = m_rightEncoderCount;
  m_prevHorizEncoderCount = m_horizEncoderCount;

  const double dLeftM = static_cast<double>(dLeftCounts) / kVertCountsPerM;
  const double dRightM = static_cast<double>(dRightCounts) / kVertCountsPerM;
  const double dHorizM = static_cast<double>(dHorizCounts) / kHorizCountsPerM;
  const double dForward = (dLeftM + dRightM) / 2.0;
  const double dTheta = m_gyroYawRateRadPerSec * kLoopPeriodSec;

  m_odomTheta = WrapAngleRadians(m_odomTheta + dTheta);
  m_odomX += dForward * std::cos(m_odomTheta) - dHorizM * std::sin(m_odomTheta);
  m_odomY += dForward * std::sin(m_odomTheta) + dHorizM * std::cos(m_odomTheta);

#if 0
// DISABLED: replaced by encoder balance loop — re-enable by removing #if 0
  double correctedOmega = m_cmdOmega;
  if (m_headingHoldActive && std::abs(m_cmdOmega) < kOmegaDeadband) {
    const double headingError = WrapAngleRadians(m_headingHoldTarget - m_odomTheta);
    const double correction = Clamp(headingError * kHeadingHoldKP,
                                    -kMaxHeadingCorrection,
                                    kMaxHeadingCorrection);
    correctedOmega += correction;
  } else {
    m_headingHoldTarget = m_odomTheta;
  }
#endif

#if 0
// DISABLED: replaced by encoder balance loop — re-enable by removing #if 0
  const double vertLeft = m_cmdVx - correctedOmega * kWheelBaseM / 2.0;
  const double vertRight = m_cmdVx + correctedOmega * kWheelBaseM / 2.0;
#endif
  const double horiz = m_cmdVy;

#if 0
// DISABLED: replaced by encoder balance loop — re-enable by removing #if 0
  const int32_t leftQpps = MetersPerSecondToCountsPerSecond(vertLeft, kVertCountsPerM);
  const int32_t leftTrimmedQpps =
      static_cast<int32_t>(std::lround(static_cast<double>(leftQpps) * kLeftMotorTrim));
  const int32_t rightQpps = MetersPerSecondToCountsPerSecond(vertRight, kVertCountsPerM);
  const int32_t horizQpps = MetersPerSecondToCountsPerSecond(horiz, kHorizCountsPerM);
#endif

  const int32_t horizQpps = MetersPerSecondToCountsPerSecond(horiz, kHorizCountsPerM);

  const double baseSpeed_M1 = m_qpps_M1 * m_driveSpeedFraction;
  const double baseSpeed_M2 = m_qpps_M2 * m_driveSpeedFraction;

  int32_t deltaLeft = m_leftEncoderCount - encLeftPrev;
  int32_t deltaRight = m_rightEncoderCount - encRightPrev;
  if (m_cmdVx < 0.0) {
    // M2 encoder counts inverted in reverse — negate to match M1 sign convention.
    deltaRight = -deltaRight;
  }
  encLeftPrev = m_leftEncoderCount;
  encRightPrev = m_rightEncoderCount;

  double normalizedLeft = static_cast<double>(deltaLeft) / m_qpps_M1;
  double normalizedRight = static_cast<double>(deltaRight) / m_qpps_M2;
  double encError = normalizedLeft - normalizedRight;
  double correction = m_encBalanceKp * encError;

  double speedM1 = baseSpeed_M1 - (correction * m_qpps_M1);
  double speedM2 = baseSpeed_M2 + (correction * m_qpps_M2);

  m_driveBalanceTuningTable->GetEntry("EncBalanceError").SetDouble(encError);
  m_driveBalanceTuningTable->GetEntry("EncBalanceCorrection").SetDouble(correction);
  m_driveBalanceTuningTable->GetEntry("SpeedCmdM1").SetDouble(speedM1);
  m_driveBalanceTuningTable->GetEntry("SpeedCmdM2").SetDouble(speedM2);
  m_driveBalanceTuningTable->GetEntry("DeltaLeft").SetDouble(static_cast<double>(deltaLeft));
  m_driveBalanceTuningTable->GetEntry("DeltaRight").SetDouble(static_cast<double>(deltaRight));

  if (m_driveOutputsEnabled) {
#if 0
// DISABLED: replaced by encoder balance loop — re-enable by removing #if 0
    m_roboclaw.SetM1M2Speed(kAddrVertical, leftTrimmedQpps, rightQpps);
#endif
    m_roboclaw.SetM1Speed(kAddrVertical, static_cast<int32_t>(speedM1));
    m_roboclaw.SetM2Speed(kAddrVertical, static_cast<int32_t>(speedM2));
    m_roboclaw.SetM1Speed(kAddrHorizontal, horizQpps);
  }

  if (++dashboardCounter >= 5) {
    dashboardCounter = 0;

    // --- Odometry ---
    frc::SmartDashboard::PutNumber("Drive/OdomX_m", m_odomX);
    frc::SmartDashboard::PutNumber("Drive/OdomY_m", m_odomY);
    frc::SmartDashboard::PutNumber("Drive/OdomTheta_deg", m_odomTheta * 180.0 / kPi);
    frc::SmartDashboard::PutNumber("Drive/AvgDistanceMeters", GetAverageDistanceMeters());

    // --- Raw encoder counts ---
    frc::SmartDashboard::PutNumber("Drive/EncLeft_counts", m_leftEncoderCount);
    frc::SmartDashboard::PutNumber("Drive/EncRight_counts", m_rightEncoderCount);
    frc::SmartDashboard::PutNumber("Drive/EncHoriz_counts", m_horizEncoderCount);

    // --- IMU ---
    frc::SmartDashboard::PutNumber("IMU/YawRate_radps", m_gyroYawRateRadPerSec);
    frc::SmartDashboard::PutNumber("IMU/YawRate_degps", m_gyroYawRateRadPerSec * 180.0 / kPi);

    // --- Current motor commands (what was sent to RoboClaw this cycle) ---
    frc::SmartDashboard::PutNumber("Drive/Cmd_Vx_mps", m_cmdVx);
    frc::SmartDashboard::PutNumber("Drive/Cmd_Vy_mps", m_cmdVy);
    frc::SmartDashboard::PutNumber("Drive/Cmd_Omega_radps", m_cmdOmega);
    frc::SmartDashboard::PutNumber("Drive/QPPS_Left",
        static_cast<double>(static_cast<int32_t>(std::lround(
            (m_cmdVx - m_cmdOmega * kWheelBaseM / 2.0) * kVertCountsPerM))));
    frc::SmartDashboard::PutNumber("Drive/QPPS_Right",
        static_cast<double>(static_cast<int32_t>(std::lround(
            (m_cmdVx + m_cmdOmega * kWheelBaseM / 2.0) * kVertCountsPerM))));
    frc::SmartDashboard::PutNumber("Drive/QPPS_Horiz",
        static_cast<double>(static_cast<int32_t>(std::lround(
            m_cmdVy * kHorizCountsPerM))));
    // Publish actual encoder velocities (counts/cycle * 50Hz = counts/sec estimate)
    // These help diagnose QPPS convergence between left and right channels.
    // dLeft/dRight/dHoriz are already computed above in Periodic() — use those values.
    frc::SmartDashboard::PutNumber("Drive/QPPS_Left_Actual",
        static_cast<double>(dLeftCounts) * 50.0);
    frc::SmartDashboard::PutNumber("Drive/QPPS_Right_Actual",
        static_cast<double>(dRightCounts) * 50.0);
    frc::SmartDashboard::PutNumber("Drive/QPPS_Horiz_Actual",
        static_cast<double>(dHorizCounts) * 50.0);
    frc::SmartDashboard::PutBoolean("Control/DriveOutputsEnabled",
                                    m_driveOutputsEnabled);

    // --- Heading hold diagnostics ---
    frc::SmartDashboard::PutNumber("Drive/HeadingHoldTarget_deg",
        m_headingHoldTarget * 180.0 / kPi);

    // --- RoboClaw health ---
    frc::SmartDashboard::PutNumber("RoboClaw/ErrorCount", 0);
  }
}

void Drivetrain::SimulationPeriodic() {}

void Drivetrain::ResetDriveEncoders() {
  m_roboclaw.ResetEncoders(kAddrVertical);
  m_roboclaw.ResetEncoders(kAddrHorizontal);

  m_leftEncoderCount = 0;
  m_rightEncoderCount = 0;
  m_horizEncoderCount = 0;
  m_prevLeftEncoderCount = 0;
  m_prevRightEncoderCount = 0;
  m_prevHorizEncoderCount = 0;
  encLeftPrev = 0;
  encRightPrev = 0;
  m_haveEncoderReference = true;

  ResetOdometry();
}

double Drivetrain::GetAverageDistanceMeters() const {
  return (static_cast<double>(m_leftEncoderCount) +
          static_cast<double>(m_rightEncoderCount)) /
         2.0 / kVertCountsPerM;
}

void Drivetrain::DriveArcade(double xSpeed, double zRotation) {
  Drive(xSpeed, 0.0, zRotation);
}

void Drivetrain::StopDrive() {
  Drive(0.0, 0.0, 0.0);
  m_roboclaw.SetM1M2Speed(kAddrVertical, 0, 0);
  m_roboclaw.SetM1Speed(kAddrHorizontal, 0);
}

void Drivetrain::SetDriveOutputsEnabled(bool enabled) {
  m_driveOutputsEnabled = enabled;
}

bool Drivetrain::GetDriveOutputsEnabled() const {
  return m_driveOutputsEnabled;
}

void Drivetrain::Drive(double vx, double vy, double omega) {
  m_cmdVx = vx;
  m_cmdVy = vy;
  m_cmdOmega = omega;
#if 0
// DISABLED: replaced by encoder balance loop — re-enable by removing #if 0
  constexpr double kCommandActivationEpsilon = 1e-3;
  if (!m_headingHoldActive &&
      (std::abs(vx) > kCommandActivationEpsilon ||
       std::abs(vy) > kCommandActivationEpsilon ||
       std::abs(omega) > kCommandActivationEpsilon)) {
    m_headingHoldActive = true;
    m_headingHoldTarget = m_odomTheta;
  }
  if (std::abs(omega) >= kOmegaDeadband) {
    m_headingHoldTarget = m_odomTheta;
  }
#endif
}

void Drivetrain::GetOdometry(double& x, double& y, double& theta) const {
  x = m_odomX;
  y = m_odomY;
  theta = m_odomTheta;
}

void Drivetrain::ResetOdometry(double x, double y, double theta) {
  m_odomX = x;
  m_odomY = y;
  m_odomTheta = WrapAngleRadians(theta);
  m_headingHoldTarget = m_odomTheta;
}

void Drivetrain::SetGyroYawRate(double yawRateRadPerSec) {
  m_gyroYawRateRadPerSec = yawRateRadPerSec;
}

double Drivetrain::GetGyroYawRate() const {
  return m_gyroYawRateRadPerSec;
}

int32_t Drivetrain::GetEncoderLeft() const {
  return m_leftEncoderCount;
}

int32_t Drivetrain::GetEncoderRight() const {
  return m_rightEncoderCount;
}

int32_t Drivetrain::GetEncoderHoriz() const {
  return m_horizEncoderCount;
}

double Drivetrain::GetHeading() const {
  return m_odomTheta;
}

bool Drivetrain::VerifyControllers() {
  const auto verticalFirmware = GetFirmwareVersionVertical();
  const auto horizontalFirmware = GetFirmwareVersionHorizontal();
  return verticalFirmware.has_value() && horizontalFirmware.has_value();
}

std::optional<std::string> Drivetrain::GetFirmwareVersionVertical() {
  return m_roboclaw.ReadFirmwareVersion(kAddrVertical);
}

std::optional<std::string> Drivetrain::GetFirmwareVersionHorizontal() {
  return m_roboclaw.ReadFirmwareVersion(kAddrHorizontal);
}

double Drivetrain::Clamp(double value, double minValue, double maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

double Drivetrain::WrapAngleRadians(double angleRad) {
  while (angleRad > kPi) {
    angleRad -= 2.0 * kPi;
  }
  while (angleRad < -kPi) {
    angleRad += 2.0 * kPi;
  }
  return angleRad;
}

int32_t Drivetrain::MetersPerSecondToCountsPerSecond(
    double speedMetersPerSecond, double countsPerMeter) {
  return static_cast<int32_t>(std::lround(speedMetersPerSecond * countsPerMeter));
}

bool Drivetrain::ConsumeDashboardButtonEdge(const char* key, bool& previousState) {
  const bool currentState = frc::SmartDashboard::GetBoolean(key, false);
  const bool risingEdge = currentState && !previousState;
  previousState = currentState;
  if (risingEdge) {
    frc::SmartDashboard::PutBoolean(key, false);
    previousState = false;
  }
  return risingEdge;
}

bool Drivetrain::PidCloseEnough(double lhs, double rhs) {
  return std::abs(lhs - rhs) <= kPidMatchTolerance;
}

bool Drivetrain::PidMatches(const VelocityPidSetting& a,
                            const VelocityPidSetting& b) {
  return PidCloseEnough(a.kp, b.kp) &&
         PidCloseEnough(a.ki, b.ki) &&
         PidCloseEnough(a.kd, b.kd) &&
         (a.qpps == b.qpps);
}

void Drivetrain::InitializePidDashboard() {
  frc::SmartDashboard::SetDefaultString("RoboClawPID/Status", "Init");
  frc::SmartDashboard::SetDefaultBoolean("RoboClawPID/LastReadOk", false);
  frc::SmartDashboard::SetDefaultBoolean("RoboClawPID/LastApplyOk", false);
  frc::SmartDashboard::SetDefaultBoolean("RoboClawPID/Dirty", false);
  frc::SmartDashboard::SetDefaultString("RoboClawPID/LastError", "");
  frc::SmartDashboard::SetDefaultBoolean("RoboClawPID/AutoRefreshOnBoot",
                                         m_autoRefreshOnBoot);
  frc::SmartDashboard::SetDefaultString("RoboClawPID/Controller_0x80/Role", "Vertical");
  frc::SmartDashboard::SetDefaultString("RoboClawPID/Controller_0x81/Role", "Horizontal");
  frc::SmartDashboard::SetDefaultString("RoboClawPID/Controller_0x81/M2/Status",
                                        "Unused");
  frc::SmartDashboard::SetDefaultBoolean("RoboClawPID/RefreshActual", false);
  frc::SmartDashboard::SetDefaultBoolean("RoboClawPID/LoadPendingFromConfig", false);
  frc::SmartDashboard::SetDefaultBoolean("RoboClawPID/LoadPendingFromActual", false);
  frc::SmartDashboard::SetDefaultBoolean("RoboClawPID/ApplyPendingToControllers", false);
  frc::SmartDashboard::SetDefaultBoolean("RoboClawPID/WriteNVM", false);
  frc::SmartDashboard::SetDefaultBoolean("RoboClawPID/ApplyAndWriteNVM", false);
  frc::SmartDashboard::SetDefaultBoolean("Drive/FullReset", false);
  m_autoRefreshOnBoot = frc::SmartDashboard::GetBoolean(
      "RoboClawPID/AutoRefreshOnBoot", m_autoRefreshOnBoot);
}

void Drivetrain::PublishPidConfigValues() const {
  frc::SmartDashboard::PutNumber(std::string{kPidController80M1} + "/Config/Kp", m_verticalM1Config.kp);
  frc::SmartDashboard::PutNumber(std::string{kPidController80M1} + "/Config/Ki", m_verticalM1Config.ki);
  frc::SmartDashboard::PutNumber(std::string{kPidController80M1} + "/Config/Kd", m_verticalM1Config.kd);
  frc::SmartDashboard::PutNumber(std::string{kPidController80M1} + "/Config/QPPS", m_verticalM1Config.qpps);

  frc::SmartDashboard::PutNumber(std::string{kPidController80M2} + "/Config/Kp", m_verticalM2Config.kp);
  frc::SmartDashboard::PutNumber(std::string{kPidController80M2} + "/Config/Ki", m_verticalM2Config.ki);
  frc::SmartDashboard::PutNumber(std::string{kPidController80M2} + "/Config/Kd", m_verticalM2Config.kd);
  frc::SmartDashboard::PutNumber(std::string{kPidController80M2} + "/Config/QPPS", m_verticalM2Config.qpps);

  frc::SmartDashboard::PutNumber(std::string{kPidController81M1} + "/Config/Kp", m_horizontalM1Config.kp);
  frc::SmartDashboard::PutNumber(std::string{kPidController81M1} + "/Config/Ki", m_horizontalM1Config.ki);
  frc::SmartDashboard::PutNumber(std::string{kPidController81M1} + "/Config/Kd", m_horizontalM1Config.kd);
  frc::SmartDashboard::PutNumber(std::string{kPidController81M1} + "/Config/QPPS", m_horizontalM1Config.qpps);
}

void Drivetrain::PublishPidPendingValues() const {
  frc::SmartDashboard::PutNumber(std::string{kPidController80M1} + "/Pending/Kp", m_verticalM1Pending.kp);
  frc::SmartDashboard::PutNumber(std::string{kPidController80M1} + "/Pending/Ki", m_verticalM1Pending.ki);
  frc::SmartDashboard::PutNumber(std::string{kPidController80M1} + "/Pending/Kd", m_verticalM1Pending.kd);
  frc::SmartDashboard::PutNumber(std::string{kPidController80M1} + "/Pending/QPPS", m_verticalM1Pending.qpps);

  frc::SmartDashboard::PutNumber(std::string{kPidController80M2} + "/Pending/Kp", m_verticalM2Pending.kp);
  frc::SmartDashboard::PutNumber(std::string{kPidController80M2} + "/Pending/Ki", m_verticalM2Pending.ki);
  frc::SmartDashboard::PutNumber(std::string{kPidController80M2} + "/Pending/Kd", m_verticalM2Pending.kd);
  frc::SmartDashboard::PutNumber(std::string{kPidController80M2} + "/Pending/QPPS", m_verticalM2Pending.qpps);

  frc::SmartDashboard::PutNumber(std::string{kPidController81M1} + "/Pending/Kp", m_horizontalM1Pending.kp);
  frc::SmartDashboard::PutNumber(std::string{kPidController81M1} + "/Pending/Ki", m_horizontalM1Pending.ki);
  frc::SmartDashboard::PutNumber(std::string{kPidController81M1} + "/Pending/Kd", m_horizontalM1Pending.kd);
  frc::SmartDashboard::PutNumber(std::string{kPidController81M1} + "/Pending/QPPS", m_horizontalM1Pending.qpps);
}

void Drivetrain::PublishPidActualValues() const {
  frc::SmartDashboard::PutNumber(std::string{kPidController80M1} + "/Actual/Kp", m_verticalM1Actual.kp);
  frc::SmartDashboard::PutNumber(std::string{kPidController80M1} + "/Actual/Ki", m_verticalM1Actual.ki);
  frc::SmartDashboard::PutNumber(std::string{kPidController80M1} + "/Actual/Kd", m_verticalM1Actual.kd);
  frc::SmartDashboard::PutNumber(std::string{kPidController80M1} + "/Actual/QPPS", m_verticalM1Actual.qpps);

  frc::SmartDashboard::PutNumber(std::string{kPidController80M2} + "/Actual/Kp", m_verticalM2Actual.kp);
  frc::SmartDashboard::PutNumber(std::string{kPidController80M2} + "/Actual/Ki", m_verticalM2Actual.ki);
  frc::SmartDashboard::PutNumber(std::string{kPidController80M2} + "/Actual/Kd", m_verticalM2Actual.kd);
  frc::SmartDashboard::PutNumber(std::string{kPidController80M2} + "/Actual/QPPS", m_verticalM2Actual.qpps);

  frc::SmartDashboard::PutNumber(std::string{kPidController81M1} + "/Actual/Kp", m_horizontalM1Actual.kp);
  frc::SmartDashboard::PutNumber(std::string{kPidController81M1} + "/Actual/Ki", m_horizontalM1Actual.ki);
  frc::SmartDashboard::PutNumber(std::string{kPidController81M1} + "/Actual/Kd", m_horizontalM1Actual.kd);
  frc::SmartDashboard::PutNumber(std::string{kPidController81M1} + "/Actual/QPPS", m_horizontalM1Actual.qpps);
}

void Drivetrain::ReadPidPendingValuesFromDashboard() {
  m_verticalM1Pending.kp = frc::SmartDashboard::GetNumber(
      std::string{kPidController80M1} + "/Pending/Kp", m_verticalM1Pending.kp);
  m_verticalM1Pending.ki = frc::SmartDashboard::GetNumber(
      std::string{kPidController80M1} + "/Pending/Ki", m_verticalM1Pending.ki);
  m_verticalM1Pending.kd = frc::SmartDashboard::GetNumber(
      std::string{kPidController80M1} + "/Pending/Kd", m_verticalM1Pending.kd);
  m_verticalM1Pending.qpps = static_cast<uint32_t>(std::lround(frc::SmartDashboard::GetNumber(
      std::string{kPidController80M1} + "/Pending/QPPS", static_cast<double>(m_verticalM1Pending.qpps))));

  m_verticalM2Pending.kp = frc::SmartDashboard::GetNumber(
      std::string{kPidController80M2} + "/Pending/Kp", m_verticalM2Pending.kp);
  m_verticalM2Pending.ki = frc::SmartDashboard::GetNumber(
      std::string{kPidController80M2} + "/Pending/Ki", m_verticalM2Pending.ki);
  m_verticalM2Pending.kd = frc::SmartDashboard::GetNumber(
      std::string{kPidController80M2} + "/Pending/Kd", m_verticalM2Pending.kd);
  m_verticalM2Pending.qpps = static_cast<uint32_t>(std::lround(frc::SmartDashboard::GetNumber(
      std::string{kPidController80M2} + "/Pending/QPPS", static_cast<double>(m_verticalM2Pending.qpps))));

  m_horizontalM1Pending.kp = frc::SmartDashboard::GetNumber(
      std::string{kPidController81M1} + "/Pending/Kp", m_horizontalM1Pending.kp);
  m_horizontalM1Pending.ki = frc::SmartDashboard::GetNumber(
      std::string{kPidController81M1} + "/Pending/Ki", m_horizontalM1Pending.ki);
  m_horizontalM1Pending.kd = frc::SmartDashboard::GetNumber(
      std::string{kPidController81M1} + "/Pending/Kd", m_horizontalM1Pending.kd);
  m_horizontalM1Pending.qpps = static_cast<uint32_t>(std::lround(frc::SmartDashboard::GetNumber(
      std::string{kPidController81M1} + "/Pending/QPPS",
      static_cast<double>(m_horizontalM1Pending.qpps))));
}

void Drivetrain::LoadPidPendingFromConfig() {
  m_verticalM1Pending = m_verticalM1Config;
  m_verticalM2Pending = m_verticalM2Config;
  m_horizontalM1Pending = m_horizontalM1Config;
}

void Drivetrain::LoadPidPendingFromActual() {
  if (!m_hasActualPidValues) {
    return;
  }
  m_verticalM1Pending = m_verticalM1Actual;
  m_verticalM2Pending = m_verticalM2Actual;
  m_horizontalM1Pending = m_horizontalM1Actual;
}

bool Drivetrain::RefreshPidActualFromControllers() {
  const auto vm1 = m_roboclaw.ReadM1VelocityPID(kAddrVertical);
  const auto vm2 = m_roboclaw.ReadM2VelocityPID(kAddrVertical);
  const auto hm1 = m_roboclaw.ReadM1VelocityPID(kAddrHorizontal);
  if (!vm1.ok || !vm2.ok || !hm1.ok) {
    m_hasActualPidValues = false;
    m_pidLastReadOk = false;
    return false;
  }

  m_verticalM1Actual.kp = vm1.pid.Kp;
  m_verticalM1Actual.ki = vm1.pid.Ki;
  m_verticalM1Actual.kd = vm1.pid.Kd;
  m_verticalM1Actual.qpps = vm1.pid.qpps;
  m_verticalM2Actual.kp = vm2.pid.Kp;
  m_verticalM2Actual.ki = vm2.pid.Ki;
  m_verticalM2Actual.kd = vm2.pid.Kd;
  m_verticalM2Actual.qpps = vm2.pid.qpps;
  m_horizontalM1Actual.kp = hm1.pid.Kp;
  m_horizontalM1Actual.ki = hm1.pid.Ki;
  m_horizontalM1Actual.kd = hm1.pid.Kd;
  m_horizontalM1Actual.qpps = hm1.pid.qpps;
  m_hasActualPidValues = true;
  m_pidLastReadOk = true;
  PublishPidActualValues();
  UpdatePidMatchAndDirtyFlags();
  return true;
}

bool Drivetrain::ApplyPidPendingToControllers(bool writeNvm) {
  const bool driveTestActive = frc::SmartDashboard::GetBoolean("DriveTest/Enable", false) ||
                               frc::SmartDashboard::GetBoolean("DriveTest/Active", false);
  if (driveTestActive) {
    SetPidStatus("Apply rejected", "Drive test is active", m_pidLastReadOk, false);
    m_pidLastApplyOk = false;
    return false;
  }
  if (frc::DriverStation::IsEnabled()) {
    SetPidStatus("Apply rejected", "Robot must be disabled for PID apply",
                 m_pidLastReadOk, false);
    m_pidLastApplyOk = false;
    return false;
  }

  StopDrive();
  SetDriveOutputsEnabled(false);

  const bool applyOk =
      m_roboclaw.SetM1VelocityPID(
          kAddrVertical,
          roboclaw::VelocityPID{m_verticalM1Pending.kp, m_verticalM1Pending.ki,
                                m_verticalM1Pending.kd, m_verticalM1Pending.qpps}) &&
      m_roboclaw.SetM2VelocityPID(
          kAddrVertical,
          roboclaw::VelocityPID{m_verticalM2Pending.kp, m_verticalM2Pending.ki,
                                m_verticalM2Pending.kd, m_verticalM2Pending.qpps}) &&
      m_roboclaw.SetM1VelocityPID(
          kAddrHorizontal,
          roboclaw::VelocityPID{m_horizontalM1Pending.kp, m_horizontalM1Pending.ki,
                                m_horizontalM1Pending.kd, m_horizontalM1Pending.qpps});

  if (!applyOk) {
    m_pidLastApplyOk = false;
    SetPidStatus("Apply failed", "Failed to write pending PID values",
                 m_pidLastReadOk, false);
    return false;
  }

  if (writeNvm) {
    const bool nvmOk =
        m_roboclaw.WriteNVM(kAddrVertical) &&
        m_roboclaw.WriteNVM(kAddrHorizontal);
    if (!nvmOk) {
      m_pidLastApplyOk = false;
      SetPidStatus("Apply failed", "PID applied to RAM but WriteNVM failed",
                   m_pidLastReadOk, false);
      return false;
    }
  }

  const bool readbackOk = RefreshPidActualFromControllers();
  m_pidLastApplyOk = readbackOk;
  if (!readbackOk) {
    SetPidStatus("Apply/readback failed",
                 "Applied values but readback failed",
                 false, false);
    return false;
  }

  SetPidStatus(writeNvm ? "Applied + NVM written" : "Applied (RAM only)", "",
               true, true);
  return true;
}

void Drivetrain::HandlePidDashboardActions() {
  if (ConsumeDashboardButtonEdge("RoboClawPID/RefreshActual", m_prevRefreshActual)) {
    if (RefreshPidActualFromControllers()) {
      SetPidStatus("Readback refreshed", "", true, m_pidLastApplyOk);
    } else {
      SetPidStatus("Readback failed", "Failed to read one or more PID slots",
                   false, m_pidLastApplyOk);
    }
  }

  if (ConsumeDashboardButtonEdge("RoboClawPID/LoadPendingFromConfig",
                                 m_prevLoadPendingFromConfig)) {
    LoadPidPendingFromConfig();
    PublishPidPendingValues();
    UpdatePidMatchAndDirtyFlags();
    SetPidStatus("Pending loaded from Config", "", m_pidLastReadOk, m_pidLastApplyOk);
  }

  if (ConsumeDashboardButtonEdge("RoboClawPID/LoadPendingFromActual",
                                 m_prevLoadPendingFromActual)) {
    if (!m_hasActualPidValues) {
      SetPidStatus("Load pending failed", "No Actual PID values are available yet",
                   m_pidLastReadOk, m_pidLastApplyOk);
    } else {
      LoadPidPendingFromActual();
      PublishPidPendingValues();
      UpdatePidMatchAndDirtyFlags();
      SetPidStatus("Pending loaded from Actual", "", m_pidLastReadOk, m_pidLastApplyOk);
    }
  }

  if (ConsumeDashboardButtonEdge("RoboClawPID/ApplyPendingToControllers", m_prevApplyPending)) {
    ApplyPidPendingToControllers(false);
  }

  if (ConsumeDashboardButtonEdge("RoboClawPID/WriteNVM", m_prevWriteNvm)) {
    const bool driveTestActive = frc::SmartDashboard::GetBoolean("DriveTest/Enable", false) ||
                                 frc::SmartDashboard::GetBoolean("DriveTest/Active", false);
    if (driveTestActive) {
      SetPidStatus("NVM write rejected", "Drive test is active", m_pidLastReadOk, m_pidLastApplyOk);
      return;
    }
    if (frc::DriverStation::IsEnabled()) {
      SetPidStatus("NVM write rejected", "Robot must be disabled for WriteNVM",
                   m_pidLastReadOk, m_pidLastApplyOk);
      return;
    }
    const bool writeOk = m_roboclaw.WriteNVM(kAddrVertical) &&
                         m_roboclaw.WriteNVM(kAddrHorizontal);
    if (writeOk) {
      const bool refreshed = RefreshPidActualFromControllers();
      SetPidStatus(refreshed ? "NVM write complete" : "NVM write complete; readback failed",
                   refreshed ? "" : "WriteNVM succeeded but readback failed",
                   refreshed, m_pidLastApplyOk);
    } else {
      SetPidStatus("NVM write failed", "WriteNVM failed", m_pidLastReadOk, m_pidLastApplyOk);
    }
  }

  if (ConsumeDashboardButtonEdge("RoboClawPID/ApplyAndWriteNVM", m_prevApplyAndWriteNvm)) {
    ApplyPidPendingToControllers(true);
  }
}

void Drivetrain::UpdatePidMatchAndDirtyFlags() {
  const bool v1Matches = m_hasActualPidValues && PidMatches(m_verticalM1Actual, m_verticalM1Config);
  const bool v2Matches = m_hasActualPidValues && PidMatches(m_verticalM2Actual, m_verticalM2Config);
  const bool h1Matches = m_hasActualPidValues && PidMatches(m_horizontalM1Actual, m_horizontalM1Config);
  frc::SmartDashboard::PutBoolean(std::string{kPidController80M1} + "/MatchesConfig", v1Matches);
  frc::SmartDashboard::PutBoolean(std::string{kPidController80M2} + "/MatchesConfig", v2Matches);
  frc::SmartDashboard::PutBoolean(std::string{kPidController81M1} + "/MatchesConfig", h1Matches);

  const bool dirty =
      !PidMatches(m_verticalM1Pending, m_verticalM1Actual) ||
      !PidMatches(m_verticalM2Pending, m_verticalM2Actual) ||
      !PidMatches(m_horizontalM1Pending, m_horizontalM1Actual);
  frc::SmartDashboard::PutBoolean("RoboClawPID/Dirty", m_hasActualPidValues && dirty);
}

void Drivetrain::SetPidStatus(const std::string& status, const std::string& lastError,
                              bool lastReadOk, bool lastApplyOk) {
  m_pidLastReadOk = lastReadOk;
  m_pidLastApplyOk = lastApplyOk;
  frc::SmartDashboard::PutBoolean("RoboClaw/PIDLoaded", m_pidLastReadOk || m_pidLastApplyOk);
  frc::SmartDashboard::PutString("RoboClawPID/Status", status);
  frc::SmartDashboard::PutBoolean("RoboClawPID/LastReadOk", m_pidLastReadOk);
  frc::SmartDashboard::PutBoolean("RoboClawPID/LastApplyOk", m_pidLastApplyOk);
  frc::SmartDashboard::PutString("RoboClawPID/LastError", lastError);
  frc::SmartDashboard::PutBoolean("RoboClawPID/AutoRefreshOnBoot", m_autoRefreshOnBoot);
}
