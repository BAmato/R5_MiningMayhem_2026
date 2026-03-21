#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/AnalogInput.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/GenericEntry.h>

class SortingSystem : public frc2::SubsystemBase {
 private:
  frc::AnalogInput m_hallSensor{0};
  nt::GenericEntry* m_voltageEntry;
  static constexpr double kGeodiniumThreshold = 2.5;  // CALIBRATE: set from real measurements

 public:
  SortingSystem();

  void Periodic() override;
  void SimulationPeriodic() override;
  double GetHallVoltage() const;
  bool IsGeodinium() const;
};
