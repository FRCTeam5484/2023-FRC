package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmAngleConstants;
import frc.robot.Constants.ArmExtensionConstants;
import frc.robot.subsystems.subArmAngle;
import frc.robot.subsystems.subArmExtension;
import frc.robot.subsystems.subPneumatic;
import frc.robot.subsystems.subSwerve;

public class cmdAutonomous_DeadCode_PlaceConeDocked extends SequentialCommandGroup {
  public cmdAutonomous_DeadCode_PlaceConeDocked(subSwerve swerve, subArmAngle angle, subArmExtension extension, subPneumatic air) {
    addCommands(
      new cmdAuto_SetGoal(angle, extension, ArmAngleConstants.MidPosition, ArmExtensionConstants.MidPosition).withTimeout(3),      
      new InstantCommand(() -> air.toggle(), air),
      new cmdAuto_SetDefault(angle, extension),
      new InstantCommand(() -> air.toggle(), air),
      new cmdAutonomous_DeadCode_CrossLineDocked(swerve, 0.35)
    );
  }
}
