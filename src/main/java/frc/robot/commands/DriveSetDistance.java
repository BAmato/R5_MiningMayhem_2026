package frc.robot.commands;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import java.lang.reflect.Field;
import java.lang.reflect.Method;

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
    getLeftEncoder().reset();
    getRightEncoder().reset();
  }

  @Override
  public void execute() {
    getDrive().arcadeDrive(-0.5, 0.0);
  }

  @Override
  public boolean isFinished() {
    double leftDistance = getLeftEncoder().getDistance();
    double rightDistance = getRightEncoder().getDistance();
    double averageDistance = (leftDistance + rightDistance) / 2.0;
    return averageDistance >= distanceMeters;
  }

  @Override
  public void end(boolean interrupted) {
    getDrive().stopMotor();
  }

  private Encoder getLeftEncoder() {
    return getEncoder("getQuadratureEncoder1", "quadratureEncoder1");
  }

  private Encoder getRightEncoder() {
    return getEncoder("getQuadratureEncoder2", "quadratureEncoder2");
  }

  private DifferentialDrive getDrive() {
    try {
      Field field = drivetrain.getClass().getField("differentialDrive1");
      Object value = field.get(drivetrain);
      if (value instanceof DifferentialDrive) {
        return (DifferentialDrive) value;
      }
    } catch (ReflectiveOperationException ignored) {
      // Fall back to getter lookup.
    }

    try {
      Method method = drivetrain.getClass().getMethod("getDifferentialDrive1");
      Object value = method.invoke(drivetrain);
      if (value instanceof DifferentialDrive) {
        return (DifferentialDrive) value;
      }
    } catch (ReflectiveOperationException ignored) {
      // Throw below if neither field nor getter are available.
    }

    throw new IllegalStateException("Unable to access drivetrain differentialDrive1");
  }

  private Encoder getEncoder(String getterName, String fieldName) {
    try {
      Method method = drivetrain.getClass().getMethod(getterName);
      Object value = method.invoke(drivetrain);
      if (value instanceof Encoder) {
        return (Encoder) value;
      }
    } catch (ReflectiveOperationException ignored) {
      // Fall back to field lookup.
    }

    try {
      Field field = drivetrain.getClass().getField(fieldName);
      Object value = field.get(drivetrain);
      if (value instanceof Encoder) {
        return (Encoder) value;
      }
    } catch (ReflectiveOperationException ignored) {
      // Throw below if neither field nor getter are available.
    }

    throw new IllegalStateException("Unable to access drivetrain encoder: " + fieldName);
  }
}
