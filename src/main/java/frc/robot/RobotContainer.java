package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.cmdTeleOp_Drive;
import frc.robot.subsystems.subArmAngle;
import frc.robot.subsystems.subArmExtension;
import frc.robot.subsystems.subDriveTrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final CommandXboxController driverOne = new CommandXboxController(OperatorConstants.DriverOne);
  private final CommandXboxController driverTwo = new CommandXboxController(OperatorConstants.DriverTwo);
  private final subDriveTrain driveTrain = new subDriveTrain();
  private final subArmAngle armAngle = new subArmAngle();
  private final subArmExtension armExtension = new subArmExtension();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    driveTrain.setDefaultCommand(new cmdTeleOp_Drive(
      driveTrain, 
      () -> -modifyAxis(driverOne.getLeftY())*DriveConstants.MaxVelocityMetersPerSecond, 
      () -> -modifyAxis(driverOne.getLeftX())*DriveConstants.MaxVelocityMetersPerSecond, 
      () -> -modifyAxis(driverOne.getRightX())*DriveConstants.MaxAngularVelocityRadiansPerSecond
    ));
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  public Command getAutonomousCommand() {
    return null;
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }
}
