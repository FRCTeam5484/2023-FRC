package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.subArmAngle;

public class cmdAuto_HoldAngle extends CommandBase {
  subArmAngle angle;
  double angleToHold;
  public cmdAuto_HoldAngle(subArmAngle angle) {
    this.angle = angle;
    addRequirements(angle);
  }

  @Override
  public void initialize() {
    angleToHold = angle.getEncoderPosition();
  }

  @Override
  public void execute() {
    System.out.println(angleToHold);
    angle.enablePID(angleToHold);
  }

  @Override
  public void end(boolean interrupted) {
    angle.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
