#include "subsystems/Drivetrain.h"

#include <cmath>

#include <frc/smartdashboard/SmartDashboard.h>

Drivetrain::Drivetrain() {
  SetName("Drivetrain");
  SetSubsystem("Drivetrain");

  ResetDriveEncoders();
}

void Drivetrain::Periodic() {
  if (const auto left = m_roboclaw.ReadM1Encoder(kAddrVertical)) {
    m_leftEncoderCount = left->count;
  }
  if (const auto right = m_roboclaw.ReadM2Encoder(kAddrVertical)) {
    m_rightEncoderCount = right->count;
  }
  if (const auto horiz = m_roboclaw.ReadM1Encoder(kAddrHorizontal)) {
    m_horizEncoderCount = horiz->count;
  }

  if (!m_haveEncoderReference) {
    m_prevLeftEncoderCount = m_leftEncoderCount;
    m_prevRightEncoderCount = m_rightEncoderCount;
    m_prevHorizEncoderCount = m_horizEncoderCount;
    m_haveEncoderReference = true;
  }

  const int32_t dLeftCounts = m_leftEncoderCount - m_prevLeftEncoderCount;
  const int32_t dRightCounts = m_rightEncoderCount - m_prevRightEncoderCount;
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

  double correctedOmega = m_cmdOmega;
  if (std::abs(m_cmdOmega) < kOmegaDeadband) {
    const double headingError = WrapAngleRadians(m_headingHoldTarget - m_odomTheta);
    const double correction = Clamp(headingError * kHeadingHoldKP,
                                    -kMaxHeadingCorrection,
                                    kMaxHeadingCorrection);
    correctedOmega += correction;
  } else {
    m_headingHoldTarget = m_odomTheta;
  }

  const double vertLeft = m_cmdVx - correctedOmega * kWheelBaseM / 2.0;
  const double vertRight = m_cmdVx + correctedOmega * kWheelBaseM / 2.0;
  const double horiz = m_cmdVy;

  const int32_t leftQpps = MetersPerSecondToCountsPerSecond(vertLeft, kVertCountsPerM);
  const int32_t rightQpps = MetersPerSecondToCountsPerSecond(vertRight, kVertCountsPerM);
  const int32_t horizQpps = MetersPerSecondToCountsPerSecond(horiz, kHorizCountsPerM);

  m_roboclaw.SetM1M2Speed(kAddrVertical, leftQpps, rightQpps);
  m_roboclaw.SetM1Speed(kAddrHorizontal, horizQpps);

  frc::SmartDashboard::PutNumber("OdomX", m_odomX);
  frc::SmartDashboard::PutNumber("OdomY", m_odomY);
  frc::SmartDashboard::PutNumber("OdomTheta", m_odomTheta * 180.0 / kPi);
  frc::SmartDashboard::PutNumber("EncLeft", m_leftEncoderCount);
  frc::SmartDashboard::PutNumber("EncRight", m_rightEncoderCount);
  frc::SmartDashboard::PutNumber("EncHoriz", m_horizEncoderCount);
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

void Drivetrain::Drive(double vx, double vy, double omega) {
  m_cmdVx = vx;
  m_cmdVy = vy;
  m_cmdOmega = omega;
  if (std::abs(omega) >= kOmegaDeadband) {
    m_headingHoldTarget = m_odomTheta;
  }
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
  const auto verticalFirmware = m_roboclaw.ReadFirmwareVersion(kAddrVertical);
  const auto horizontalFirmware = m_roboclaw.ReadFirmwareVersion(kAddrHorizontal);
  return verticalFirmware.has_value() && horizontalFirmware.has_value();
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
