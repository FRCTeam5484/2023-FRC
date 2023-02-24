package frc.robot.commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.subArmExtension;

public class cmdArmExtension_TeleOp extends CommandBase {
  subArmExtension extension;
  DoubleSupplier extensionPower;
  BooleanSupplier overrideSafety;
  public cmdArmExtension_TeleOp(subArmExtension extension, DoubleSupplier extensionPower, BooleanSupplier overrideSafety) {
    this.extension = extension;
    this.extensionPower = extensionPower;
    this.overrideSafety = overrideSafety;
    addRequirements(extension);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    extension.teleOp(extensionPower.getAsDouble(), overrideSafety.getAsBoolean());
  }

  @Override
  public void end(boolean interrupted) {
    extension.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
