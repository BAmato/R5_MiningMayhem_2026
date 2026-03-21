#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/AnalogInput.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/GenericEntry.h>

class SortingSystem : public frc2::SubsystemBase {
 private:
  frc::AnalogInput m_hallSensor{0};
  nt::GenericEntry* m_voltageEntry = nullptr;
  // CALIBRATE both values by measuring real sensor readings.
  // Geodinium: contains neodymium magnets — reliably elevated voltage.
  // Nebulite: plain PLA — lower but above the noise floor.
  // Procedure: hold each material type near the sensor, read "Hall/Voltage"
  // on SmartDashboard, average 10 readings for each. Set kGeodiniumThreshold
  // midway between Nebulite peak and Geodinium minimum. Set kNebuliteMinVoltage
  // just above the idle noise floor (robot running, no material present).
  static constexpr double kGeodiniumThreshold = 2.5;  // CALIBRATE
  static constexpr double kNebuliteMinVoltage = 0.5;  // CALIBRATE: noise floor

 public:
  SortingSystem();

  void Periodic() override;
  void SimulationPeriodic() override;
  double GetHallVoltage() const;
  bool IsGeodinium() const;
  bool IsNebulite() const;   // true if voltage is above noise floor but below Geodinium threshold
};
