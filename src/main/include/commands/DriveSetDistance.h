// ROBOTBUILDER TYPE: Command.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

class DriveSetDistance : public frc2::CommandHelper<frc2::Command, DriveSetDistance> {
 public:
  explicit DriveSetDistance(double distanceMeters);

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;
  bool RunsWhenDisabled() const override;

 private:
  double m_distanceMeters;
};
