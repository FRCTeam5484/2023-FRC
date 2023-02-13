package frc.robot;

import frc.robot.Constants.ArmAngleConstants;
import frc.robot.Constants.ArmExtensionConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ServoConstants;
import frc.robot.commands.cmdAuto_SetGoal;
import frc.robot.commands.cmdClaw_Actuate;
import frc.robot.commands.cmdTeleOp_ArmAngle;
import frc.robot.commands.cmdTeleOp_ArmExtension;
import frc.robot.commands.cmdTeleOp_Drive;
import frc.robot.commands.cmdTeleOp_ItemNeeded;
import frc.robot.subsystems.subArmAngle;
import frc.robot.subsystems.subArmExtension;
import frc.robot.subsystems.subClaw;
import frc.robot.subsystems.subDriveTrain;
import frc.robot.subsystems.subItemNeeded;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final CommandXboxController driverOne = new CommandXboxController(OperatorConstants.DriverOne);
  private final CommandXboxController driverTwo = new CommandXboxController(OperatorConstants.DriverTwo);
  private final subDriveTrain driveTrain = new subDriveTrain();
  private final subArmAngle armAngle = new subArmAngle();
  private final subArmExtension armExtension = new subArmExtension();
  private final subClaw claw = new subClaw();
  private final subItemNeeded item = new subItemNeeded();
  SendableChooser<Command> chooser = new SendableChooser<>();

  public RobotContainer() {
    configureDriverOne();
    configureDriverTwo();
    addAutoOptions();
  }

  private void addAutoOptions(){
    chooser.addOption("Cross Line", loadPathPlannerTrajectoryToRamseteCommand("/Users/roger.johnson/Documents/GitHub/2023-FRC/src/main/deploy/deploy/pathplanner/CrossLine.path"));
    chooser.addOption("Cross and Dock", loadPathPlannerTrajectoryToRamseteCommand("/Users/roger.johnson/Documents/GitHub/2023-FRC/src/main/deploy/deploy/pathplanner/CrossLineLevel.path"));
    Shuffleboard.getTab("Autonomous").add(chooser);
  }

  private void configureDriverOne() {
    driveTrain.setDefaultCommand(new cmdTeleOp_Drive(
      driveTrain, 
      () -> -modifyAxis(driverOne.getLeftY())*DriveConstants.MaxVelocityMetersPerSecond, 
      () -> modifyAxis(driverOne.getLeftX())*DriveConstants.MaxVelocityMetersPerSecond, 
      () -> modifyAxis(driverOne.getRightX())*DriveConstants.MaxAngularVelocityRadiansPerSecond
    ));
    claw.setDefaultCommand(new cmdClaw_Actuate(
      claw, 
      () -> modifyAxis(driverOne.getLeftTriggerAxis()) , 
      () -> -modifyAxis(driverOne.getRightTriggerAxis())
    ));
    driverOne.a().onTrue(new cmdTeleOp_ItemNeeded(item, ServoConstants.cubeDown));
    driverOne.b().onTrue(new cmdTeleOp_ItemNeeded(item, ServoConstants.cubeUp));
    driverOne.x().onTrue(new cmdTeleOp_ItemNeeded(item, ServoConstants.coneDown));
    driverOne.x().onTrue(new cmdTeleOp_ItemNeeded(item, ServoConstants.coneUp));
  }

  private void configureDriverTwo() {
    armAngle.setDefaultCommand(new cmdTeleOp_ArmAngle(armAngle, () -> -modifyAxis(driverTwo.getLeftY())*ArmAngleConstants.PowerFactor));
    armExtension.setDefaultCommand(new cmdTeleOp_ArmExtension(armExtension, () -> -modifyAxis(driverTwo.getRightY())*ArmExtensionConstants.PowerFactor));

    driverTwo.y().whileTrue(new cmdAuto_SetGoal(armAngle, armExtension, ArmAngleConstants.HighPosition, ArmExtensionConstants.HighPosition));
    driverTwo.b().whileTrue(new cmdAuto_SetGoal(armAngle, armExtension, ArmAngleConstants.MidPosition, ArmExtensionConstants.MidPosition));
    driverTwo.a().whileTrue(new cmdAuto_SetGoal(armAngle, armExtension, ArmAngleConstants.GroundPosition, ArmExtensionConstants.GroundPosition));
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
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

  public Command loadPathPlannerTrajectoryToRamseteCommand(String fileName){
    /* Trajectory trajectory;
    try{
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(fileName);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    }catch(IOException exception){
      DriverStation.reportError("Unable to open trajectory " + fileName, exception.getStackTrace());
      System.out.println("Unable to open trajectory " + fileName);
      return new InstantCommand();
    }

    RamseteCommand ramseteCommand = new RamseteCommand(
      trajectory, 
      driveTrain::getPosition, 
      new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta), 
      new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter), 
      driveTrain.m_kinematics, null, null, null, null, null) */
      return new InstantCommand();
  }
}
