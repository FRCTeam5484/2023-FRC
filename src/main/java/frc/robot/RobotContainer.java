package frc.robot;

import frc.robot.Constants.ArmAngleConstants;
import frc.robot.Constants.ArmExtensionConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ServoConstants;
import frc.robot.classes.AutonomousCommands;
import frc.robot.commands.cmdAuto_SetGoal;
import frc.robot.commands.cmdAutonomous_DeadCode_CrossLine;
import frc.robot.commands.cmdAutonomous_DeadCode_CrossLineDocked;
import frc.robot.commands.cmdAutonomous_DeadCode_PlaceConeCrossLine;
import frc.robot.commands.cmdAutonomous_DeadCode_PlaceConeDocked;
import frc.robot.commands.cmdClaw_TeleOp;
import frc.robot.commands.cmdArmAngle_TeleOp;
import frc.robot.commands.cmdArmExtension_TeleOp;
import frc.robot.commands.cmdAuto_SetDefault;
import frc.robot.commands.cmdSwerve_TeleOp;
import frc.robot.subsystems.subArmAngle;
import frc.robot.subsystems.subArmExtension;
import frc.robot.subsystems.subClaw;
import frc.robot.subsystems.subItemNeeded;
import frc.robot.subsystems.subLimeLight;
import frc.robot.subsystems.subSwerve;
import frc.robot.subsystems.subItemNeeded.itemList;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final AutonomousCommands autoOptions = new AutonomousCommands();

  public RobotContainer() {
    configureDriverOne();
    configureDriverTwo();
    addAutoOptions();
  }

  private void addAutoOptions(){
    chooser.setDefaultOption("Deadcode Crossline", new cmdAutonomous_DeadCode_CrossLine(swerve, 0.35));
    chooser.addOption("Deadcode Crossline Docked", new cmdAutonomous_DeadCode_CrossLineDocked(swerve, 0.35));
    chooser.addOption("DeadCode Place Cone, Cross Line", new cmdAutonomous_DeadCode_PlaceConeCrossLine(swerve, armAngle, armExtension, claw));
    chooser.addOption("DeadCode Place Cone, Docked", new cmdAutonomous_DeadCode_PlaceConeDocked(swerve, armAngle, armExtension, claw));
    //chooser.addOption("Cross Line", autoOptions.CrossLine(swerve));
    //chooser.addOption("Place Cone, Cross Line", autoOptions.PlaceConeCrossLine(swerve, armAngle, armExtension, claw));
    SmartDashboard.putData("Auto Options", chooser);
  }

  private void configureDriverOne() {
    swerve.setDefaultCommand(
      new cmdSwerve_TeleOp(
          swerve,
          () -> MathUtil.applyDeadband(-driverOne.getLeftY(), 0.01),
          () -> MathUtil.applyDeadband(driverOne.getLeftX(), 0.01),
          () -> MathUtil.applyDeadband(-driverOne.getRightX(), 0.01)));
    driverOne.x().onTrue(new InstantCommand(() -> item.setCurrentSelection(itemList.CubeDown)));
    driverOne.b().onTrue(new InstantCommand(() -> item.setCurrentSelection(itemList.CubeUp)));
    driverOne.a().onTrue(new InstantCommand(() -> item.setCurrentSelection(itemList.ConeDown)));
    driverOne.y().onTrue(new InstantCommand(() -> item.setCurrentSelection(itemList.ConeUp)));
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
    driverTwo.leftBumper().whileFalse(new InstantCommand(() -> claw.stop()));
    driverTwo.rightBumper().whileTrue(new cmdClaw_TeleOp(claw, () -> 0, () -> 1, () -> true));
    driverTwo.rightBumper().whileFalse(new InstantCommand(() -> claw.stop()));
    
    driverTwo.povUp().whileTrue(new cmdArmExtension_TeleOp(armExtension, () -> ArmExtensionConstants.PowerFactor, () -> true));
    driverTwo.povUp().whileFalse(new InstantCommand(() -> armExtension.stop()));
    driverTwo.povDown().whileTrue(new cmdArmExtension_TeleOp(armExtension, () -> -ArmExtensionConstants.PowerFactor, () -> true));
    driverTwo.povDown().whileFalse(new InstantCommand(() -> armExtension.stop()));

    driverTwo.povLeft().whileTrue(new cmdArmAngle_TeleOp(armAngle, () -> ArmAngleConstants.PowerFactor, () -> true));
    driverTwo.povLeft().whileFalse(new InstantCommand(() -> armAngle.stop()));
    driverTwo.povRight().whileTrue(new cmdArmAngle_TeleOp(armAngle, () -> -ArmAngleConstants.PowerFactor, () -> true));
    driverTwo.povRight().whileFalse(new InstantCommand(() -> armAngle.stop()));
    
    driverTwo.back().onTrue(new InstantCommand(() -> claw.resetPosition()));
    driverTwo.start().onTrue(new InstantCommand(() -> armExtension.resetPosition()));

    driverTwo.x().whileTrue(new cmdAuto_SetGoal(armAngle, armExtension, ArmAngleConstants.HumanFeedPosition, ArmExtensionConstants.HumanFeedPosition));
    driverTwo.x().whileFalse(new cmdAuto_SetDefault(armAngle, armExtension));
    driverTwo.y().whileTrue(new cmdAuto_SetGoal(armAngle, armExtension, ArmAngleConstants.HighPosition, ArmExtensionConstants.HighPosition));
    driverTwo.y().whileFalse(new cmdAuto_SetDefault(armAngle, armExtension));
    driverTwo.b().whileTrue(new cmdAuto_SetGoal(armAngle, armExtension, ArmAngleConstants.MidPosition, ArmExtensionConstants.MidPosition));
    driverTwo.b().whileFalse(new cmdAuto_SetDefault(armAngle, armExtension));
    driverTwo.a().whileTrue(new cmdAuto_SetGoal(armAngle, armExtension, ArmAngleConstants.GroundPosition, ArmExtensionConstants.GroundPosition));
    driverTwo.a().whileFalse(new cmdAuto_SetDefault(armAngle, armExtension));
  }

  public Command getAutonomousCommand() {
    try { return chooser.getSelected(); } 
    catch (NullPointerException ex) { 
      DriverStation.reportError("auto choose NULL somewhere in getAutonomousCommand in RobotContainer.java", null);
      return new InstantCommand();
    }
  }
}