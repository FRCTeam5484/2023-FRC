package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.subArmAngle;

public class cmdTeleOp_ArmAngle extends CommandBase {
  subArmAngle angle;
  double angleCommand;
  public cmdTeleOp_ArmAngle(subArmAngle angle, double angleCommand) {
    this.angle = angle;
    this.angleCommand = angleCommand;
    addRequirements(angle);
  }
  
  @Override
  public void initialize() {}
  
  @Override
  public void execute() {
    angle.teleOp(angleCommand);
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
