package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.subItemNeeded.itemList;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
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
  public final subItemNeeded item = new subItemNeeded();
  public final subPneumatic air = new subPneumatic();
  SendableChooser<Command> chooser = new SendableChooser<>();
  //private final AutonomousCommands autoOptions = new AutonomousCommands();

  public RobotContainer() {
    configureDriverOne();
    configureDriverTwo();
    addAutoOptions();
  }

  private void addAutoOptions(){
    chooser.setDefaultOption("Crossline", new cmdAutonomous_DeadCode_CrossLine(swerve, 0.35));
    chooser.addOption("Crossline Docked", new cmdAutonomous_DeadCode_CrossLineDocked(swerve, 0.35));
    chooser.addOption("Place Cone, Cross Line", new cmdAutonomous_DeadCode_PlaceConeCrossLine(swerve, armAngle, armExtension, air));
    chooser.addOption("Place Cone, Cross Line, Grab Cube", new cmdAutonomous_DeadCode_PlaceConeCrossLineGrabCube(swerve, armAngle, armExtension, air));
    chooser.addOption("Place Cone, Docked", new cmdAutonomous_DeadCode_PlaceConeDocked(swerve, armAngle, armExtension, air));
    chooser.addOption("Place Cube, Docked", new cmdAutonomous_DeadCode_PlaceCubeDocked(swerve, armAngle, armExtension, air));
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
          () -> MathUtil.applyDeadband(-driverOne.getRightX(), 0.01),
          () -> driverOne.rightTrigger().getAsBoolean(),
          () -> driverOne.leftTrigger().getAsBoolean()));
    driverOne.x().onTrue(new InstantCommand(() -> item.setCurrentSelection(itemList.CubeDown)));
    driverOne.b().onTrue(new InstantCommand(() -> item.setCurrentSelection(itemList.CubeUp)));
    driverOne.a().onTrue(new InstantCommand(() -> item.setCurrentSelection(itemList.ConeDown)));
    driverOne.y().onTrue(new InstantCommand(() -> item.setCurrentSelection(itemList.ConeUp)));
    driverOne.leftBumper().whileTrue(new RunCommand(() -> swerve.setXMode()));

    //driverOne.rightTrigger().onTrue(new cmdAuto_Level(swerve));
  }

  private void configureDriverTwo() {
    armAngle.setDefaultCommand(new cmdArmAngle_TeleOp(armAngle, () -> MathUtil.applyDeadband(-driverTwo.getLeftY()*ArmAngleConstants.PowerFactor, 0.01), () -> false));
    armExtension.setDefaultCommand(new cmdArmExtension_TeleOp(armExtension, () -> MathUtil.applyDeadband(-driverTwo.getRightY()*ArmExtensionConstants.PowerFactor, 0.01), () -> false));

    driverTwo.leftBumper().onTrue(new cmdAuto_OpenPartly(air));
    driverTwo.rightBumper().onTrue(new InstantCommand(() -> air.toggle()));
    
    driverTwo.povUp().whileTrue(new cmdArmExtension_TeleOp(armExtension, () -> ArmExtensionConstants.PowerFactor, () -> true));
    driverTwo.povUp().whileFalse(new InstantCommand(() -> armExtension.stop()));
    driverTwo.povDown().whileTrue(new cmdArmExtension_TeleOp(armExtension, () -> -ArmExtensionConstants.PowerFactor, () -> true));
    driverTwo.povDown().whileFalse(new InstantCommand(() -> armExtension.stop()));

    driverTwo.povLeft().whileTrue(new cmdArmAngle_TeleOp(armAngle, () -> ArmAngleConstants.PowerFactor, () -> true));
    driverTwo.povLeft().whileFalse(new InstantCommand(() -> armAngle.stop()));
    driverTwo.povRight().whileTrue(new cmdArmAngle_TeleOp(armAngle, () -> -ArmAngleConstants.PowerFactor, () -> true));
    driverTwo.povRight().whileFalse(new InstantCommand(() -> armAngle.stop()));
    
    driverTwo.back().onTrue(new InstantCommand(() -> armExtension.resetPosition()));
    driverTwo.start().onTrue(new cmdAuto_OpenPartly(air));

    driverTwo.x().whileTrue(new cmdAuto_SetGoal(armAngle, armExtension, ArmAngleConstants.HumanFeedPosition, ArmExtensionConstants.HumanFeedPosition));
    driverTwo.x().whileFalse(new cmdAuto_SetDefault(armAngle, armExtension));
    driverTwo.y().whileTrue(new cmdAuto_SetGoal(armAngle, armExtension, ArmAngleConstants.HighPosition, ArmExtensionConstants.HighPosition));
    driverTwo.y().whileFalse(new cmdAuto_SetDefault(armAngle, armExtension));
    driverTwo.b().whileTrue(new cmdAuto_SetGoal(armAngle, armExtension, ArmAngleConstants.MidPosition, ArmExtensionConstants.MidPosition));
    driverTwo.b().whileFalse(new cmdAuto_SetGoal(armAngle, armExtension, ArmAngleConstants.MidPlacement, ArmExtensionConstants.MidPosition));
    driverTwo.a().whileTrue(new cmdAuto_SetGoal(armAngle, armExtension, ArmAngleConstants.HumanSlidePosition, ArmExtensionConstants.HumanSlidePosition));
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