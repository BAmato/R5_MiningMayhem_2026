# R5 Mining Mayhem 2026 Code Reference

This repository contains a RobotBuilder-generated WPILib C++ robot project. The sections below summarize the currently implemented classes, functions, and how they behave based on the code in `src/main`.

## Robot Lifecycle (Robot)

**Location:** `src/main/include/Robot.h`, `src/main/cpp/Robot.cpp`

- `Robot::Robot()` initializes LiveWindow support for test mode and reports RobotBuilder usage to the HAL. It also stores a `RobotContainer` singleton pointer.  
- `Robot::RobotPeriodic()` runs the command scheduler every robot packet.  
- `Robot::DisabledInit()` / `Robot::DisabledPeriodic()` are empty placeholders for disabled-mode setup and periodic actions.  
- `Robot::AutonomousInit()` requests the autonomous command from the container and schedules it if present.  
- `Robot::AutonomousPeriodic()` is currently empty.  
- `Robot::TeleopInit()` cancels the current autonomous command (if any) and clears the pointer.  
- `Robot::TeleopPeriodic()` is currently empty.  
- `Robot::TestPeriodic()` is currently empty.  
- `main()` starts the robot via `frc::StartRobot<Robot>()` when not running tests.  

## Robot Container (RobotContainer)

**Location:** `src/main/include/RobotContainer.h`, `src/main/cpp/RobotContainer.cpp`

- `RobotContainer::GetInstance()` implements a singleton and lazily constructs the container.  
- `RobotContainer::RobotContainer()` constructs subsystems, publishes the drivetrain to SmartDashboard, adds an `AutonomousCommand` to the dashboard, configures button bindings, and sets up the autonomous chooser with a default `AutonomousCommand`.  
- `RobotContainer::ConfigureButtonBindings()` is currently empty, reserved for controller/button setup.  
- `RobotContainer::GetAutonomousCommand()` returns the currently selected autonomous command from the chooser.  

**Subsystem members:**
- `m_drivetrain` (drivetrain hardware + helpers)  
- `m_sortingSystem` (hall sensor subsystem)  
- `m_intake` (intake subsystem placeholder)  
- `m_latch` (latch subsystem placeholder)  

## Commands

### AutonomousCommand

**Location:** `src/main/include/commands/AutonomousCommand.h`, `src/main/cpp/commands/AutonomousCommand.cpp`

- `AutonomousCommand::AutonomousCommand()` sets the command name and is ready to declare subsystem requirements.  
- `Initialize()`, `Execute()`, and `End()` are empty and ready for autonomous logic.  
- `IsFinished()` always returns `false`, so the command never ends on its own.  
- `RunsWhenDisabled()` returns `false`.  

### DriveSetDistance

**Location:** `src/main/include/commands/DriveSetDistance.h`, `src/main/cpp/commands/DriveSetDistance.cpp`

- `DriveSetDistance::DriveSetDistance(double distanceMeters)` stores the target distance, names the command, and requires the drivetrain subsystem.  
- `Initialize()` resets drivetrain encoders.  
- `Execute()` drives the robot forward in arcade mode at -0.5 speed with zero rotation.  
- `IsFinished()` returns true once the average encoder distance meets/exceeds the target distance.  
- `End(bool interrupted)` stops the drivetrain.  
- `RunsWhenDisabled()` returns `false`.  

## Subsystems

### Drivetrain

**Location:** `src/main/include/subsystems/Drivetrain.h`, `src/main/cpp/subsystems/Drivetrain.cpp`

- Hardware: two PWM VictorSPX motor controllers, differential drive, and three quadrature encoders.  
- `Drivetrain::Drivetrain()` configures subsystem name, adds hardware to LiveWindow, sets safety and encoder distance-per-pulse values.  
- `Periodic()` / `SimulationPeriodic()` are empty placeholders.  
- `ResetDriveEncoders()` resets the first two encoders used for distance.  
- `GetAverageDistanceMeters()` averages the distances from encoder 1 and 2.  
- `DriveArcade(double xSpeed, double zRotation)` drives the differential drive in arcade mode.  
- `StopDrive()` stops the differential drive motors.  

### Intake

**Location:** `src/main/include/subsystems/Intake.h`, `src/main/cpp/subsystems/Intake.cpp`

- `Intake::Intake()` sets the subsystem name.  
- `Periodic()` / `SimulationPeriodic()` are empty placeholders for intake logic.  

### SortingSystem

**Location:** `src/main/include/subsystems/SortingSystem.h`, `src/main/cpp/subsystems/SortingSystem.cpp`

- Hardware: analog hall sensor on channel 0.  
- `SortingSystem::SortingSystem()` sets the subsystem name and registers the hall sensor.  
- `SortingSystem::SortingSystem()` publishes the hall sensor voltage to the “Sorting System” Shuffleboard tab.  
- `Periodic()` updates the Shuffleboard entry with the latest hall sensor voltage; `SimulationPeriodic()` is still a placeholder.  

### Latch

**Location:** `src/main/include/subsystems/Latch.h`, `src/main/cpp/subsystems/Latch.cpp`

- `Latch::Latch()` sets the subsystem name.  
- `Periodic()` / `SimulationPeriodic()` are empty placeholders for latch logic.  
