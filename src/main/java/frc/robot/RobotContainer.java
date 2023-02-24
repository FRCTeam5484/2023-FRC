package frc.robot;

import frc.robot.Constants.ArmAngleConstants;
import frc.robot.Constants.ArmExtensionConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ServoConstants;
import frc.robot.commands.cmdAuto_SetGoal;
import frc.robot.commands.cmdClaw_TeleOp;
import frc.robot.commands.cmdItemNeeded_TeleOp;
import frc.robot.commands.cmdArmAngle_TeleOp;
import frc.robot.commands.cmdArmExtension_TeleOp;
import frc.robot.commands.cmdSwerve_TeleOp;
import frc.robot.subsystems.subArmAngle;
import frc.robot.subsystems.subArmExtension;
import frc.robot.subsystems.subClaw;
import frc.robot.subsystems.subItemNeeded;
import frc.robot.subsystems.subLimeLight;
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
  public final subSwerve swerve = new subSwerve();
  public final subArmAngle armAngle = new subArmAngle();
  public final subArmExtension armExtension = new subArmExtension();
  public final subClaw claw = new subClaw();
  public final subItemNeeded item = new subItemNeeded();
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
      new cmdSwerve_TeleOp(
          swerve,
          () -> MathUtil.applyDeadband(-driverOne.getLeftY(), 0.01),
          () -> MathUtil.applyDeadband(driverOne.getLeftX(), 0.01),
          () -> MathUtil.applyDeadband(-driverOne.getRightX(), 0.01),
          () -> true));
    driverOne.a().onTrue(new cmdItemNeeded_TeleOp(item, ServoConstants.cubeDown));
    driverOne.b().onTrue(new cmdItemNeeded_TeleOp(item, ServoConstants.cubeUp));
    driverOne.x().onTrue(new cmdItemNeeded_TeleOp(item, ServoConstants.coneDown));
    driverOne.y().onTrue(new cmdItemNeeded_TeleOp(item, ServoConstants.coneUp)); 
    driverOne.leftBumper().whileTrue(new RunCommand(() -> swerve.setXMode()));
  }

  private void configureDriverTwo() {
    armAngle.setDefaultCommand(new cmdArmAngle_TeleOp(armAngle, () -> MathUtil.applyDeadband(-driverTwo.getLeftY()*ArmAngleConstants.PowerFactor, 0.01), () -> false));
    armExtension.setDefaultCommand(new cmdArmExtension_TeleOp(armExtension, () -> MathUtil.applyDeadband(-driverTwo.getRightY()*ArmExtensionConstants.PowerFactor, 0.01), () -> false));
    claw.setDefaultCommand(new cmdClaw_TeleOp(
      claw, 
      () -> MathUtil.applyDeadband(driverTwo.getLeftTriggerAxis(), 0.1), 
      () -> MathUtil.applyDeadband(driverTwo.getRightTriggerAxis(), 0.1),
      () -> false
    ));

    driverTwo.leftBumper().whileTrue(new cmdClaw_TeleOp(claw, () -> 1, () -> 0, () -> true));
    driverTwo.leftBumper().whileFalse(new cmdClaw_TeleOp(claw, () -> 0, () -> 0, () -> true));
    driverTwo.rightBumper().whileTrue(new cmdClaw_TeleOp(claw, () -> 0, () -> 1, () -> true));
    driverTwo.rightBumper().whileFalse(new cmdClaw_TeleOp(claw, () -> 0, () -> 0, () -> true));
    
    driverTwo.povUp().onTrue(new cmdArmExtension_TeleOp(armExtension, () -> ArmExtensionConstants.PowerFactor, () -> true));
    driverTwo.povUp().onFalse(new cmdArmExtension_TeleOp(armExtension, () -> 0, () -> true));
    driverTwo.povDown().onTrue(new cmdArmExtension_TeleOp(armExtension, () -> -ArmExtensionConstants.PowerFactor, () -> true));
    driverTwo.povDown().onFalse(new cmdArmExtension_TeleOp(armExtension, () -> 0, () -> true));

    driverTwo.povLeft().onTrue(new cmdArmAngle_TeleOp(armAngle, () -> ArmAngleConstants.PowerFactor, () -> true));
    driverTwo.povLeft().onFalse(new cmdArmAngle_TeleOp(armAngle, () -> 0, () -> true));
    driverTwo.povRight().onTrue(new cmdArmAngle_TeleOp(armAngle, () -> -ArmAngleConstants.PowerFactor, () -> true));
    driverTwo.povRight().onFalse(new cmdArmAngle_TeleOp(armAngle, () -> 0, () -> true));
    
    driverTwo.back().onTrue(new InstantCommand(() -> claw.resetPosition()));
    driverTwo.start().onTrue(new InstantCommand(() -> armExtension.resetPosition()));

    driverTwo.x().whileTrue(new cmdAuto_SetGoal(armAngle, armExtension, ArmAngleConstants.HumanFeedPosition, ArmExtensionConstants.HumanFeedPosition));
    driverTwo.x().whileFalse(new cmdAuto_SetGoal(armAngle, armExtension, ArmAngleConstants.DefaultPosition, ArmExtensionConstants.DefaultPosition));
    driverTwo.y().whileTrue(new cmdAuto_SetGoal(armAngle, armExtension, ArmAngleConstants.HighPosition, ArmExtensionConstants.HighPosition));
    driverTwo.y().whileFalse(new cmdAuto_SetGoal(armAngle, armExtension, ArmAngleConstants.DefaultPosition, ArmExtensionConstants.DefaultPosition));
    driverTwo.b().whileTrue(new cmdAuto_SetGoal(armAngle, armExtension, ArmAngleConstants.MidPosition, ArmExtensionConstants.MidPosition));
    driverTwo.b().whileFalse(new cmdAuto_SetGoal(armAngle, armExtension, ArmAngleConstants.DefaultPosition, ArmExtensionConstants.DefaultPosition));
    driverTwo.a().whileTrue(new cmdAuto_SetGoal(armAngle, armExtension, ArmAngleConstants.GroundPosition, ArmExtensionConstants.GroundPosition));
    driverTwo.a().whileFalse(new cmdAuto_SetGoal(armAngle, armExtension, ArmAngleConstants.DefaultPosition, ArmExtensionConstants.DefaultPosition));
  }

  public Command getAutonomousCommand() {
    try { return chooser.getSelected(); } 
    catch (NullPointerException ex) { 
      DriverStation.reportError("auto choose NULL somewhere in getAutonomousCommand in RobotContainer.java", null);
      return new InstantCommand();
    }
  }
}