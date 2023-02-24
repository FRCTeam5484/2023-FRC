package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.subArmAngle;

public class cmdArmAngle_TeleOpSafe extends CommandBase {
  subArmAngle angle;
  DoubleSupplier anglePower;
  public cmdArmAngle_TeleOpSafe(subArmAngle angle, DoubleSupplier anglePower) {
    this.angle = angle;
    this.anglePower = anglePower;
    addRequirements(angle);
  }
  
  @Override
  public void initialize() {}
  
  @Override
  public void execute() {
    angle.teleOp(anglePower.getAsDouble());
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
