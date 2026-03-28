#pragma once

#include "RoboClawDriver.h"

#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>

#include <array>
#include <string>

namespace roboclaw {

static constexpr int kNumActiveChannels = 3;

struct ChannelDef {
  uint8_t address;
  int channel;
};

inline constexpr std::array<ChannelDef, kNumActiveChannels> kActiveChannels = {{
    {0x80, 1},
    {0x80, 2},
    {0x81, 1},
}};

class RoboClawPIDManager {
 public:
  RoboClawPIDManager(
      RoboClawDriver* driver,
      const std::array<VelocityPID, kNumActiveChannels>& defaults);

  void Initialize();
  void Periodic(bool robotEnabled);

 private:
  struct ChannelEntries {
    nt::DoubleEntry cfgKp, cfgKi, cfgKd;
    nt::IntegerEntry cfgQpps;
    nt::DoubleEntry penKp, penKi, penKd;
    nt::IntegerEntry penQpps;
    nt::DoubleEntry actKp, actKi, actKd;
    nt::IntegerEntry actQpps;
    nt::BooleanEntry actReadOk;
    nt::BooleanEntry matchesConfig;
  };

  RoboClawDriver* m_driver;
  nt::NetworkTableInstance m_nt;
  std::array<VelocityPID, kNumActiveChannels> m_configPIDs;
  std::array<ChannelEntries, kNumActiveChannels> m_channels;

  nt::StringEntry m_status;
  nt::BooleanEntry m_lastReadOk, m_lastApplyOk;
  nt::StringEntry m_lastError;
  nt::BooleanEntry m_dirty, m_pidLoaded;

  nt::BooleanEntry m_trigRefresh, m_trigLoadFromConfig, m_trigLoadFromActual;
  nt::BooleanEntry m_trigApply, m_trigWriteNVM, m_trigApplyAndWriteNVM;

  std::string ChannelBasePath(int idx) const;
  void RegisterChannelEntries(int idx);
  void PublishConfig(int idx);
  void LoadPendingFromConfig(int idx);
  bool RefreshActual(int idx);
  void RefreshAllActual();
  VelocityPID GetPending(int idx) const;
  bool ApplyPendingToRAM(int idx);
  bool ApplyAllPendingToRAM();
  bool WriteNVMForAddress(uint8_t address);
  void UpdateMatchFlags();
  void SetError(const std::string& msg);
  static bool PIDsMatch(const VelocityPID& a, const VelocityPID& b,
                        double tol = 0.001);
};

}  // namespace roboclaw
