#include "subsystems/SortingSystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

SortingSystem::SortingSystem() {
  SetName("SortingSystem");
  SetSubsystem("SortingSystem");

  auto& tab = frc::Shuffleboard::GetTab("Sorting System");
  m_voltageEntry = tab.Add("Hall Voltage", 0.0).GetEntry();
}

void SortingSystem::Periodic() {
  const double voltage = m_hallSensor.GetVoltage();

  // Update Shuffleboard "Sorting System" tab
  if (m_voltageEntry != nullptr) {
    m_voltageEntry->SetDouble(voltage);
  }

  // Also publish to SmartDashboard for easy access without opening Shuffleboard
  frc::SmartDashboard::PutNumber("Hall/Voltage", voltage);

  // Human-readable material detection state
  const char* stateStr = "EMPTY";
  if (voltage > kGeodiniumThreshold) {
    stateStr = "GEODINIUM";
  } else if (voltage > kNebuliteMinVoltage) {
    stateStr = "NEBULITE";
  }
  frc::SmartDashboard::PutString("Hall/State", stateStr);
  frc::SmartDashboard::PutBoolean("Hall/IsGeodinium", voltage > kGeodiniumThreshold);
  frc::SmartDashboard::PutBoolean("Hall/IsNebulite",
      voltage > kNebuliteMinVoltage && voltage <= kGeodiniumThreshold);
}

void SortingSystem::SimulationPeriodic() {}

double SortingSystem::GetHallVoltage() const {
  return m_hallSensor.GetVoltage();
}

bool SortingSystem::IsGeodinium() const {
  return GetHallVoltage() > kGeodiniumThreshold;
}

bool SortingSystem::IsNebulite() const {
  const double v = GetHallVoltage();
  return v > kNebuliteMinVoltage && v <= kGeodiniumThreshold;
}
