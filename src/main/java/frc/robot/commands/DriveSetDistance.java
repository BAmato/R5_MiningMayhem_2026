package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class DriveSetDistance extends CommandBase {
  private final Drivetrain drivetrain;
  private final double distanceMeters;

  public DriveSetDistance(double distanceMeters) {
    this.drivetrain = RobotContainer.m_drivetrain;
    this.distanceMeters = distanceMeters;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.getQuadratureEncoder1().reset();
    drivetrain.getQuadratureEncoder2().reset();
  }

  @Override
  public void execute() {
    drivetrain.differentialDrive1.tankDrive(-0.5, -0.5);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.differentialDrive1.stopMotor();
  }

  @Override
  public boolean isFinished() {
    double leftDistance = drivetrain.getQuadratureEncoder1().getDistance();
    double rightDistance = drivetrain.getQuadratureEncoder2().getDistance();
    double averageDistance = (leftDistance + rightDistance) / 2.0;

    return averageDistance >= distanceMeters;
  }
}
