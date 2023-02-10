package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.subArmExtension;

public class cmdTeleOp_ArmExtension extends CommandBase {
  subArmExtension extension;
  double extensionCommand;
  public cmdTeleOp_ArmExtension(subArmExtension extension, double extensionCommand) {
    this.extension = extension;
    this.extensionCommand = extensionCommand;
    addRequirements(extension);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    extension.teleOp(extensionCommand);
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
