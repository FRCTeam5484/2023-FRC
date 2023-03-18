package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;

public class cmdAutonomous_DeadCode_PlaceConeCrossLine extends SequentialCommandGroup {
  public cmdAutonomous_DeadCode_PlaceConeCrossLine(subSwerve swerve, subArmAngle angle, subArmExtension extension, subPneumatic air) {
    addCommands(
      new cmdAuto_SetGoal(angle, extension, ArmAngleConstants.MidPosition, ArmExtensionConstants.MidPosition).withTimeout(2),  
      new cmdAuto_SetGoal(angle, extension, ArmAngleConstants.MidPlacement, ArmExtensionConstants.MidPosition).withTimeout(1),         
      new InstantCommand(() -> air.open(), air),
      new WaitCommand(0.25),
      new ParallelCommandGroup(
        new cmdAuto_SetDefault(angle, extension),
        new cmdAutonomous_DeadCode_CrossLine(swerve, 0.35) 
      ),
      new cmdAuto_TurnAround(swerve)
      //new InstantCommand(() -> swerve.setXMode(), swerve)
    );
  }
}