// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

// Link test

Robot::Robot() {}
void Robot::RobotInit() {
  m_enableTestModeChooser.SetDefaultOption("Disabled", false);
  m_enableTestModeChooser.AddOption("Enabled", true);
  frc::SmartDashboard::PutData("Enable Test Mode", &m_enableTestModeChooser);
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}
// Placeholder: add autonomous periodic logic here.
void Robot::AutonomousPeriodic() {
  if (m_enableTestModeChooser.GetSelected()) {
    m_autoState = AutoState::TEST_MODE;
  } else if (m_autoState == AutoState::TEST_MODE) {
    m_autoState = AutoState::START;
  }

  switch (m_autoState) {
    case AutoState::START:
      // Initialize sensors and read NetworkTables for field setup.
      m_autoState = AutoState::SWEEP;
      break;
    case AutoState::SWEEP:
      // Default behavior: follow a hardcoded snake path to collect items.
      break;
    case AutoState::RETRIEVE_BOX:
      // Interrupt: approach and latch onto a box detected by vision.
      break;
    case AutoState::DUMP_MATERIALS:
      // Interrupt: return to start and empty the bins.
      break;
    case AutoState::TEST_MODE:
      // Debug-only calls: uncomment a single action to test hardware.
      // testMotor();
      // driveOneMeter();
      break;
  }
}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  constexpr bool kMainEntryTest = true;
  static_assert(kMainEntryTest, "Main entry test failed.");
  (void)kMainEntryTest;
  return frc::StartRobot<Robot>();
}
#endif
