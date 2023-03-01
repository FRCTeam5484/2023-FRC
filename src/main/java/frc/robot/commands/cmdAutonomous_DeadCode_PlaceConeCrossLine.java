package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmAngleConstants;
import frc.robot.Constants.ArmExtensionConstants;
import frc.robot.subsystems.subArmAngle;
import frc.robot.subsystems.subArmExtension;
import frc.robot.subsystems.subClaw;
import frc.robot.subsystems.subSwerve;

public class cmdAutonomous_DeadCode_PlaceConeCrossLine extends SequentialCommandGroup {
  public cmdAutonomous_DeadCode_PlaceConeCrossLine(subSwerve swerve, subArmAngle angle, subArmExtension extension, subClaw claw) {
    addCommands(
      new cmdAuto_SetGoal(angle, extension, ArmAngleConstants.MidPosition, ArmExtensionConstants.MidPosition).withTimeout(3),
      new RunCommand(() -> claw.openClaw(true), claw).withTimeout(1),
      new cmdAuto_SetDefault(angle, extension),
      new cmdAutonomous_DeadCode_CrossLine(swerve, 0.3)
    );
  }
}
