package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.subArmAngle;
import frc.robot.subsystems.subArmExtension;

public class cmdAuto_SetGoal extends CommandBase {
  subArmAngle angle;
  subArmExtension extension;
  double angleGoal;
  double extensionGoal;
  public cmdAuto_SetGoal(subArmAngle angle, subArmExtension extension, double highposition, double highposition2) {
    this.angle = angle;
    this.extension = extension;
    this.angleGoal = highposition;
    this.extensionGoal = highposition2;
    addRequirements(angle, extension);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    angle.enablePID(angleGoal);
    extension.enablePID(extensionGoal);
  }

  @Override
  public void end(boolean interrupted) {
    angle.stop();
    extension.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
