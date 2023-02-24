package frc.robot.commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.subArmAngle;

public class cmdArmAngle_TeleOp extends CommandBase {
  subArmAngle angle;
  DoubleSupplier anglePower;
  BooleanSupplier overrideSafety;
  public cmdArmAngle_TeleOp(subArmAngle angle, DoubleSupplier anglePower, BooleanSupplier overrideSafety) {
    this.angle = angle;
    this.anglePower = anglePower;
    this.overrideSafety = overrideSafety;
    addRequirements(angle);
  }
  
  @Override
  public void initialize() {}
  
  @Override
  public void execute() {
    angle.teleOp(anglePower.getAsDouble(), overrideSafety.getAsBoolean());
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
