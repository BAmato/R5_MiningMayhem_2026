#include "RoboClawPIDManager.h"

#include <cmath>
#include <exception>
#include <iomanip>
#include <sstream>
#include <vector>

namespace roboclaw {

RoboClawPIDManager::RoboClawPIDManager(
    RoboClawDriver* driver,
    const std::array<VelocityPID, kNumActiveChannels>& defaults)
    : m_driver(driver), m_configPIDs(defaults) {}

std::string RoboClawPIDManager::ChannelBasePath(int idx) const {
  std::ostringstream oss;
  oss << "RoboClawPID/Controller_0x" << std::uppercase << std::hex << std::setw(2)
      << std::setfill('0') << static_cast<int>(kActiveChannels[idx].address) << "/"
      << (kActiveChannels[idx].channel == 1 ? "M1" : "M2");
  return oss.str();
}

void RoboClawPIDManager::RegisterChannelEntries(int idx) {
  const std::string base = ChannelBasePath(idx) + "/";
  auto& ch = m_channels[idx];

  ch.cfgKp = m_nt.GetDoubleTopic(base + "Config/Kp").GetEntry(0.0);
  ch.cfgKi = m_nt.GetDoubleTopic(base + "Config/Ki").GetEntry(0.0);
  ch.cfgKd = m_nt.GetDoubleTopic(base + "Config/Kd").GetEntry(0.0);
  ch.cfgQpps = m_nt.GetIntegerTopic(base + "Config/QPPS").GetEntry(0);

  ch.penKp = m_nt.GetDoubleTopic(base + "Pending/Kp").GetEntry(0.0);
  ch.penKi = m_nt.GetDoubleTopic(base + "Pending/Ki").GetEntry(0.0);
  ch.penKd = m_nt.GetDoubleTopic(base + "Pending/Kd").GetEntry(0.0);
  ch.penQpps = m_nt.GetIntegerTopic(base + "Pending/QPPS").GetEntry(0);

  ch.actKp = m_nt.GetDoubleTopic(base + "Actual/Kp").GetEntry(0.0);
  ch.actKi = m_nt.GetDoubleTopic(base + "Actual/Ki").GetEntry(0.0);
  ch.actKd = m_nt.GetDoubleTopic(base + "Actual/Kd").GetEntry(0.0);
  ch.actQpps = m_nt.GetIntegerTopic(base + "Actual/QPPS").GetEntry(0);
  ch.actReadOk = m_nt.GetBooleanTopic(base + "Actual/LastReadOk").GetEntry(false);

  ch.matchesConfig = m_nt.GetBooleanTopic(base + "MatchesConfig").GetEntry(false);
}

void RoboClawPIDManager::PublishConfig(int idx) {
  const auto& pid = m_configPIDs[idx];
  auto& ch = m_channels[idx];
  ch.cfgKp.Set(pid.Kp);
  ch.cfgKi.Set(pid.Ki);
  ch.cfgKd.Set(pid.Kd);
  ch.cfgQpps.Set(pid.qpps);
}

void RoboClawPIDManager::LoadPendingFromConfig(int idx) {
  auto& ch = m_channels[idx];
  ch.penKp.Set(ch.cfgKp.Get());
  ch.penKi.Set(ch.cfgKi.Get());
  ch.penKd.Set(ch.cfgKd.Get());
  ch.penQpps.Set(ch.cfgQpps.Get());
}

bool RoboClawPIDManager::RefreshActual(int idx) {
  const auto def = kActiveChannels[idx];
  PIDReadResult result =
      (def.channel == 1) ? m_driver->ReadM1VelocityPID(def.address)
                         : m_driver->ReadM2VelocityPID(def.address);

  auto& ch = m_channels[idx];
  if (result.ok) {
    ch.actKp.Set(result.pid.Kp);
    ch.actKi.Set(result.pid.Ki);
    ch.actKd.Set(result.pid.Kd);
    ch.actQpps.Set(result.pid.qpps);
    ch.actReadOk.Set(true);
    return true;
  }

  ch.actReadOk.Set(false);
  const std::string current = m_lastError.Get("");
  const std::string extra = ChannelBasePath(idx) + ": " + result.error;
  m_lastError.Set(current.empty() ? extra : (current + " | " + extra));
  return false;
}

void RoboClawPIDManager::RefreshAllActual() {
  bool allOk = true;
  m_lastError.Set("");
  for (int idx = 0; idx < kNumActiveChannels; ++idx) {
    allOk = RefreshActual(idx) && allOk;
  }

  m_lastReadOk.Set(allOk);
  m_pidLoaded.Set(allOk);
  if (allOk) {
    m_status.Set("Actual PID read OK");
    m_lastError.Set("");
  }
  UpdateMatchFlags();
}

VelocityPID RoboClawPIDManager::GetPending(int idx) const {
  const auto& ch = m_channels[idx];
  return VelocityPID{.Kp = ch.penKp.Get(),
                     .Ki = ch.penKi.Get(),
                     .Kd = ch.penKd.Get(),
                     .qpps = static_cast<uint32_t>(ch.penQpps.Get())};
}

bool RoboClawPIDManager::ApplyPendingToRAM(int idx) {
  const auto def = kActiveChannels[idx];
  const auto pending = GetPending(idx);
  return (def.channel == 1) ? m_driver->SetM1VelocityPID(def.address, pending)
                            : m_driver->SetM2VelocityPID(def.address, pending);
}

bool RoboClawPIDManager::ApplyAllPendingToRAM() {
  bool allOk = true;
  for (int idx = 0; idx < kNumActiveChannels; ++idx) {
    allOk = ApplyPendingToRAM(idx) && allOk;
  }
  return allOk;
}

bool RoboClawPIDManager::WriteNVMForAddress(uint8_t address) {
  return m_driver->WriteNVM(address);
}

bool RoboClawPIDManager::PIDsMatch(const VelocityPID& a, const VelocityPID& b,
                                   double tol) {
  return std::fabs(a.Kp - b.Kp) < tol && std::fabs(a.Ki - b.Ki) < tol &&
         std::fabs(a.Kd - b.Kd) < tol && a.qpps == b.qpps;
}

void RoboClawPIDManager::UpdateMatchFlags() {
  bool dirty = false;
  for (int idx = 0; idx < kNumActiveChannels; ++idx) {
    auto& ch = m_channels[idx];
    if (!ch.actReadOk.Get(false)) {
      ch.matchesConfig.Set(false);
      continue;
    }

    VelocityPID actual{.Kp = ch.actKp.Get(),
                       .Ki = ch.actKi.Get(),
                       .Kd = ch.actKd.Get(),
                       .qpps = static_cast<uint32_t>(ch.actQpps.Get())};
    VelocityPID config = m_configPIDs[idx];
    VelocityPID pending = GetPending(idx);

    ch.matchesConfig.Set(PIDsMatch(actual, config));
    if (!PIDsMatch(pending, actual)) {
      dirty = true;
    }
  }
  m_dirty.Set(dirty);
}

void RoboClawPIDManager::SetError(const std::string& msg) {
  m_lastError.Set(msg);
  m_status.Set("Error: " + msg);
}

void RoboClawPIDManager::Initialize() {
  m_nt = nt::NetworkTableInstance::GetDefault();

  m_status = m_nt.GetStringTopic("RoboClawPID/Status").GetEntry("");
  m_lastReadOk = m_nt.GetBooleanTopic("RoboClawPID/LastReadOk").GetEntry(false);
  m_lastApplyOk = m_nt.GetBooleanTopic("RoboClawPID/LastApplyOk").GetEntry(false);
  m_lastError = m_nt.GetStringTopic("RoboClawPID/LastError").GetEntry("");
  m_dirty = m_nt.GetBooleanTopic("RoboClawPID/Dirty").GetEntry(false);
  m_pidLoaded = m_nt.GetBooleanTopic("RoboClaw/PIDLoaded").GetEntry(false);

  m_trigRefresh = m_nt.GetBooleanTopic("RoboClawPID/RefreshActual").GetEntry(false);
  m_trigLoadFromConfig =
      m_nt.GetBooleanTopic("RoboClawPID/LoadPendingFromConfig").GetEntry(false);
  m_trigLoadFromActual =
      m_nt.GetBooleanTopic("RoboClawPID/LoadPendingFromActual").GetEntry(false);
  m_trigApply =
      m_nt.GetBooleanTopic("RoboClawPID/ApplyPendingToControllers").GetEntry(false);
  m_trigWriteNVM = m_nt.GetBooleanTopic("RoboClawPID/WriteNVM").GetEntry(false);
  m_trigApplyAndWriteNVM =
      m_nt.GetBooleanTopic("RoboClawPID/ApplyAndWriteNVM").GetEntry(false);

  m_nt.GetStringTopic("RoboClawPID/Controller_0x80/Role").GetEntry("").Set("Vertical");
  m_nt.GetStringTopic("RoboClawPID/Controller_0x81/Role")
      .GetEntry("")
      .Set("Horizontal");
  m_nt.GetStringTopic("RoboClawPID/Controller_0x81/M2/Status")
      .GetEntry("")
      .Set("Unused");

  for (int idx = 0; idx < kNumActiveChannels; ++idx) RegisterChannelEntries(idx);
  for (int idx = 0; idx < kNumActiveChannels; ++idx) PublishConfig(idx);
  for (int idx = 0; idx < kNumActiveChannels; ++idx) LoadPendingFromConfig(idx);

  bool bootReadOk = true;
  try {
    RefreshAllActual();
  } catch (const std::exception& e) {
    bootReadOk = false;
    m_status.Set("Boot readback failed");
    m_lastError.Set(std::string("Init readback exception: ") + e.what());
    m_lastReadOk.Set(false);
    m_pidLoaded.Set(false);
  }
  if (bootReadOk) {
    m_status.Set("Initialized");
  }
}

void RoboClawPIDManager::Periodic(bool robotEnabled) {
  if (m_trigRefresh.Get(false)) {
    m_trigRefresh.Set(false);
    RefreshAllActual();
  }

  if (m_trigLoadFromConfig.Get(false)) {
    m_trigLoadFromConfig.Set(false);
    for (int idx = 0; idx < kNumActiveChannels; ++idx) {
      LoadPendingFromConfig(idx);
    }
    UpdateMatchFlags();
  }

  if (m_trigLoadFromActual.Get(false)) {
    m_trigLoadFromActual.Set(false);
    for (int idx = 0; idx < kNumActiveChannels; ++idx) {
      auto& ch = m_channels[idx];
      if (ch.actReadOk.Get(false)) {
        ch.penKp.Set(ch.actKp.Get());
        ch.penKi.Set(ch.actKi.Get());
        ch.penKd.Set(ch.actKd.Get());
        ch.penQpps.Set(ch.actQpps.Get());
      }
    }
    UpdateMatchFlags();
  }

  const bool applyRequested = m_trigApply.Get(false);
  const bool writeRequested = m_trigWriteNVM.Get(false);
  const bool applyWriteRequested = m_trigApplyAndWriteNVM.Get(false);

  if (robotEnabled) {
    if (applyRequested || writeRequested || applyWriteRequested) {
      m_trigApply.Set(false);
      m_trigWriteNVM.Set(false);
      m_trigApplyAndWriteNVM.Set(false);
      SetError("Apply/Write rejected: robot is enabled");
    }
    return;
  }

  if (applyRequested) {
    m_trigApply.Set(false);
    const bool ok = ApplyAllPendingToRAM();
    m_lastApplyOk.Set(ok);
    RefreshAllActual();
    if (ok) {
      m_lastError.Set("");
      m_status.Set("Applied to RAM");
    } else {
      SetError("One or more channels failed to apply");
    }
  }

  if (writeRequested) {
    m_trigWriteNVM.Set(false);
    const bool ok = WriteNVMForAddress(0x80) && WriteNVMForAddress(0x81);
    m_lastApplyOk.Set(ok);
    RefreshAllActual();
    if (ok) {
      m_lastError.Set("");
      m_status.Set("NVM written");
    } else {
      SetError("NVM write failed");
    }
  }

  if (applyWriteRequested) {
    m_trigApplyAndWriteNVM.Set(false);
    const bool applyOk = ApplyAllPendingToRAM();
    const bool nvmOk = applyOk && WriteNVMForAddress(0x80) && WriteNVMForAddress(0x81);
    m_lastApplyOk.Set(applyOk && nvmOk);
    RefreshAllActual();
    if (applyOk && nvmOk) {
      m_lastError.Set("");
      m_status.Set("Applied + NVM written");
    } else {
      SetError("ApplyAndWriteNVM partial failure");
    }
  }
}

}  // namespace roboclaw
