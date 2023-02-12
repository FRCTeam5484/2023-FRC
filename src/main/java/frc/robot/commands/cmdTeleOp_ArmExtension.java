package frc.robot.commands;


import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.subArmExtension;

public class cmdTeleOp_ArmExtension extends CommandBase {
  subArmExtension extension;
  DoubleSupplier extensionPower;
  public cmdTeleOp_ArmExtension(subArmExtension extension, DoubleSupplier extensionPower) {
    this.extension = extension;
    this.extensionPower = extensionPower;
    addRequirements(extension);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    extension.teleOp(extensionPower.getAsDouble());
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
