#include "subsystems/SortingSystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

SortingSystem::SortingSystem() {
  SetName("SortingSystem");
  SetSubsystem("SortingSystem");

  auto& tab = frc::Shuffleboard::GetTab("Sorting System");
  m_voltageEntry = &tab.Add("Hall Voltage", 0.0).GetEntry();
}

void SortingSystem::Periodic() {
  m_voltageEntry->SetDouble(m_hallSensor.GetVoltage());
}

void SortingSystem::SimulationPeriodic() {}

double SortingSystem::GetHallVoltage() const {
  return m_hallSensor.GetVoltage();
}

bool SortingSystem::IsGeodinium() const {
  return GetHallVoltage() > kGeodiniumThreshold;
}
