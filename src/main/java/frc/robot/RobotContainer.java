package frc.robot;

import frc.robot.Constants.ArmAngleConstants;
import frc.robot.Constants.ArmExtensionConstants;
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
import frc.robot.subsystems.subItemNeeded;
import frc.robot.subsystems.subLimeLight;
import frc.robot.subsystems.subPneumaticSystem;
import frc.robot.subsystems.subSwerve;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final CommandXboxController driverOne = new CommandXboxController(OperatorConstants.DriverOne);
  private final CommandXboxController driverTwo = new CommandXboxController(OperatorConstants.DriverTwo);
  private final subSwerve swerve = new subSwerve();
  private final subArmAngle armAngle = new subArmAngle();
  private final subArmExtension armExtension = new subArmExtension();
  private final subClaw claw = new subClaw();
  private final subItemNeeded item = new subItemNeeded();
  //private final subPneumaticSystem air = new subPneumaticSystem();
  private final subLimeLight lime = new subLimeLight();
  SendableChooser<Command> chooser = new SendableChooser<>();

  public RobotContainer() {
    configureDriverOne();
    configureDriverTwo();
    addAutoOptions();
  }

  private void addAutoOptions(){
    try{
      //chooser.setDefaultOption("Cross Line", swerve.followPathCmd("CrossLine"));
      //chooser.addOption("Cross and Dock", swerve.followPathCmd("CrossLineLevel"));
      Shuffleboard.getTab("Autonomous").add(chooser);
    }
    catch(NullPointerException ex){
      chooser.setDefaultOption("NULL Nothing", new InstantCommand());
      DriverStation.reportError("Auto Chooser NULL - Fix It", null);
    }
  }

  private void configureDriverOne() {
    swerve.setDefaultCommand(
      new cmdTeleOp_Drive(
          swerve,
          () -> MathUtil.applyDeadband(-driverOne.getLeftY(), 0.01),
          () -> MathUtil.applyDeadband(driverOne.getLeftX(), 0.01),
          () -> MathUtil.applyDeadband(-driverOne.getRightX(), 0.01),
          () -> true));
    driverOne.a().onTrue(new cmdTeleOp_ItemNeeded(item, ServoConstants.cubeDown));
    driverOne.b().onTrue(new cmdTeleOp_ItemNeeded(item, ServoConstants.cubeUp));
    driverOne.x().onTrue(new cmdTeleOp_ItemNeeded(item, ServoConstants.coneDown));
    driverOne.y().onTrue(new cmdTeleOp_ItemNeeded(item, ServoConstants.coneUp)); 
    driverOne.leftBumper().whileTrue(new RunCommand(() -> swerve.setXMode()));
  }

  private void configureDriverTwo() {
    armAngle.setDefaultCommand(new cmdTeleOp_ArmAngle(armAngle, () -> MathUtil.applyDeadband(driverTwo.getLeftY()*ArmAngleConstants.PowerFactor, 0.01)));
    armExtension.setDefaultCommand(new cmdTeleOp_ArmExtension(armExtension, () -> MathUtil.applyDeadband(driverTwo.getRightY()*ArmExtensionConstants.PowerFactor, 0.01)));
    claw.setDefaultCommand(new cmdClaw_Actuate(
      claw, 
      () -> MathUtil.applyDeadband(driverTwo.getLeftTriggerAxis(), 0.1), 
      () -> MathUtil.applyDeadband(driverTwo.getRightTriggerAxis(), 0.1)
    ));
    //driverTwo.rightBumper().whileTrue(new cmdAuto_HoldAngle(armAngle));
    //driverTwo.rightTrigger().onTrue(new InstantCommand(() -> air.toggle()));
    driverTwo.x().whileTrue(new cmdAuto_SetGoal(armAngle, armExtension, ArmAngleConstants.HumanFeedPosition, ArmExtensionConstants.HumanFeedPosition));
    driverTwo.y().whileTrue(new cmdAuto_SetGoal(armAngle, armExtension, ArmAngleConstants.HighPosition, ArmExtensionConstants.HighPosition));
    driverTwo.b().whileTrue(new cmdAuto_SetGoal(armAngle, armExtension, ArmAngleConstants.MidPosition, ArmExtensionConstants.MidPosition));
    driverTwo.a().whileTrue(new cmdAuto_SetGoal(armAngle, armExtension, ArmAngleConstants.GroundPosition, ArmExtensionConstants.GroundPosition));
  }

  public Command getAutonomousCommand() {
    try { return chooser.getSelected(); } 
    catch (NullPointerException ex) { 
      DriverStation.reportError("auto choose NULL somewhere in getAutonomousCommand in RobotContainer.java", null);
      return new InstantCommand();
    }
  }
}