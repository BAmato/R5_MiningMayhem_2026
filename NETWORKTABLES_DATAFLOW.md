# Mining Mayhem 2026 — NetworkTables Dataflow and Glass Expectations

This document explains how telemetry is produced in the current robot code, what keys/values should appear in NetworkTables (NT) when viewed in Glass, and what subsystems/processes must be operating to actually see live updates.

---

## 1) High-level dataflow

At runtime, data moves through these layers:

1. **Hardware + Jetson inputs** are sampled in subsystem `Periodic()` methods and in `Robot::RobotPeriodic()`.
2. The code publishes values using:
   - `frc::SmartDashboard::Put*` → NT keys under `/SmartDashboard/...`
   - `Shuffleboard` entries (`Sorting System` tab) → NT keys under `/Shuffleboard/...`
3. **NT server** runs on the roboRIO process with your robot program.
4. **Glass** connects as an NT client and subscribes to those keys.

If your code is running but you only see no data in Glass, the problem is usually one of:
- robot program not actually running (or repeatedly crashing/restarting),
- Glass connected to the wrong host/team,
- no loop execution (subsystem periodic path not being reached),
- keys are in `/SmartDashboard` or `/Shuffleboard` and you are looking in a different table path.

---

## 2) What code currently publishes to NetworkTables

### A) Published at startup / init

These should appear soon after robot program startup:

- `RoboClaw/OK` (bool)
- `RoboClaw/Status` (string)
- `RoboClaw/Firmware_0x80` (string)
- `RoboClaw/Firmware_0x81` (string)
- `RoboClaw/PIDLoaded` (bool)
- `Auto Mode` (sendable chooser)
- Drivetrain sendable object from `PutData(&m_drivetrain)`

---

### B) Published continuously in `Robot::RobotPeriodic()`

Every robot packet (20ms loop), these are updated:

- `Control/JetsonConnected` (bool)
- `Control/SoftwareEnable` (bool)
- `Control/ShouldDrive` (bool)
- `Control/DriveOutputsEnabled` (bool)
- `Control/StartupInhibit` (bool)
- `RoboClaw/OK` (bool)

Every 5 loops (~100ms), these are also updated:

- `Match/TimeRemainingSec` (number)
- `Servo/BeaconArmPos` (number)
- `Servo/ContainerArmPos` (number)
- `Servo/SortGatePos` (number)

---

### C) Published in `SerialBridgeSubsystem::Periodic()`

Every loop while sockets are ready:

- `Bridge/JetsonConnected` (bool)
- `Bridge/TxSeq` (number)
- `Bridge/RxCount` (number)
- `Bridge/DroppedPkts` (number)
- `Bridge/Cmd_Vx` (number)
- `Bridge/Cmd_Vy` (number)
- `Bridge/Cmd_Omega` (number)
- `Bridge/BeaconArmPos` (number)
- `Bridge/ContainerArmPos` (number)
- `Bridge/SortGatePos` (number)
- `Bridge/StartSignalFwd` (bool)

Connection logic expectations:

- `Bridge/JetsonConnected` is **true only if** at least one valid RX packet has been received and the last one is newer than 200ms.
- RX packet validity checks include:
  - 30-byte size,
  - magic bytes `0x5A 0xA5`,
  - CRC8 correctness.

So, being able to **ping** the Jetson does *not* prove this bridge is receiving valid command packets.

---

### D) Published in `Drivetrain::Periodic()`

Every 5 loops (~100ms), these are updated:

- `Drive/OdomX_m` (m)
- `Drive/OdomY_m` (m)
- `Drive/OdomTheta_deg` (deg)
- `Drive/AvgDistanceMeters` (m)
- `Drive/EncLeft_counts`
- `Drive/EncRight_counts`
- `Drive/EncHoriz_counts`
- `IMU/YawRate_radps`
- `IMU/YawRate_degps`
- `Drive/Cmd_Vx_mps`
- `Drive/Cmd_Vy_mps`
- `Drive/Cmd_Omega_radps`
- `Drive/QPPS_Left`
- `Drive/QPPS_Right`
- `Drive/QPPS_Horiz`
- `Control/DriveOutputsEnabled` (bool)
- `Drive/HeadingHoldTarget_deg`
- `RoboClaw/ErrorCount`

---

### E) Published in `SortingSystem::Periodic()`

Every loop:

- SmartDashboard:
  - `Hall/Voltage` (volts)
  - `Hall/State` (`EMPTY`, `NEBULITE`, `GEODINIUM`)
  - `Hall/IsGeodinium` (bool)
  - `Hall/IsNebulite` (bool)
- Shuffleboard tab `Sorting System`:
  - `Hall Voltage`

Thresholds in code:

- `kGeodiniumThreshold = 2.5 V`
- `kNebuliteMinVoltage = 0.5 V`

Interpretation:

- `voltage <= 0.5` → `EMPTY`
- `0.5 < voltage <= 2.5` → `NEBULITE`
- `voltage > 2.5` → `GEODINIUM`

---

## 3) Expected table locations in Glass

In the NT tree, expect:

- Most keys under **`/SmartDashboard/...`** (from `SmartDashboard::Put*`).
- Sorting tab value under **`/Shuffleboard/Sorting System/Hall Voltage`**.

If you only inspect a custom table and not `/SmartDashboard`, it can look like “no data” even when publishing is working.

---

## 4) Systems that must be operational to see data flow

To see live values in Glass, all of the following must be true:

1. **Robot code is running on roboRIO** (FRC user program active, not crashed).
2. **Main loop is executing** (`RobotPeriodic` runs).
3. **CommandScheduler runs** each loop, so subsystem `Periodic()` methods execute.
4. **NetworkTables server is reachable** from your driver-station laptop.
5. **Glass is connected to the correct robot/team/host**.
6. For `Bridge/*` and `Control/JetsonConnected` specifically: Jetson must send valid packets that match struct size/magic/CRC and timing.

If (1)-(5) are good, you should still see many keys even with Jetson disconnected (`Bridge/JetsonConnected=false` etc.).

---

## 5) Why ping success can coexist with no NT data

`ping` only verifies IP reachability. It does **not** guarantee:

- robot application loop is running,
- NT client/server connection is established,
- UDP ports/protocol for Jetson command packets are correct,
- packet format and CRC are valid.

So “ping works with 0% loss” is useful, but insufficient for NT telemetry diagnosis.

---

## 6) Quick troubleshooting checklist (current implementation)

1. In Driver Station, confirm robot code is enabled/disabled but **not crashing**.
2. In Glass, connect by team number and inspect `/SmartDashboard` directly.
3. Check for startup keys first:
   - `RoboClaw/Status`
   - `RoboClaw/Firmware_0x80`
   - `RoboClaw/Firmware_0x81`
4. Check rapidly changing counters:
   - `Bridge/TxSeq` should increment continuously.
   - `Bridge/RxCount` increments only on valid Jetson packets.
5. If `Bridge/TxSeq` moves but no keys appear, Glass is likely connected to wrong target.
6. If keys appear but `Bridge/JetsonConnected` stays false:
   - verify Jetson sends to roboRIO UDP port `5801`,
   - verify packet length/magic/CRC matches `JetsonToRioPacket`.
7. Verify your roboRIO and laptop are on the same FRC network and no firewall is blocking NT traffic.

---

## 7) Operational expectations summary

Minimum expected NT activity even with Jetson offline:

- `Control/*` booleans update,
- `Drive/*` and `IMU/*` numbers update (at ~10 Hz publish cadence),
- `Hall/*` values update,
- `Bridge/TxSeq` increments,
- `Bridge/JetsonConnected` stays false until valid RX appears.

If none of the above show in Glass, focus first on robot-program state and Glass connection target rather than Jetson ping status.
