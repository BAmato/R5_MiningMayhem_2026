#include "commands/DriveSetDistance.h"

#include "RobotContainer.h"

DriveSetDistance::DriveSetDistance(double distanceMeters)
    : m_distanceMeters(distanceMeters) {
  SetName("DriveSetDistance");
  AddRequirements(&RobotContainer::GetInstance()->m_drivetrain);
}

void DriveSetDistance::Initialize() {
  RobotContainer::GetInstance()->m_drivetrain.ResetDriveEncoders();
}

void DriveSetDistance::Execute() {
  RobotContainer::GetInstance()->m_drivetrain.DriveArcade(-0.5, 0.0);
}

bool DriveSetDistance::IsFinished() {
  return RobotContainer::GetInstance()->m_drivetrain.GetAverageDistanceMeters() >=
         m_distanceMeters;
}

void DriveSetDistance::End(bool interrupted) {
  RobotContainer::GetInstance()->m_drivetrain.StopDrive();
}

bool DriveSetDistance::RunsWhenDisabled() const {
  return false;
}
