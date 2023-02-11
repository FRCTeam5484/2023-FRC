package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.subArmAngle;

public class cmdTeleOp_ArmAngle extends CommandBase {
  subArmAngle angle;
  XboxController driver;
  public cmdTeleOp_ArmAngle(subArmAngle angle, XboxController driver) {
    this.angle = angle;
    this.driver = driver;
    addRequirements(angle);
  }
  
  @Override
  public void initialize() {}
  
  @Override
  public void execute() {
    angle.teleOp(-driver.getRightY());
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
