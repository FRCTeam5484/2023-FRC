package frc.robot.commands;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.subArmExtension;

public class cmdTeleOp_ArmExtension extends CommandBase {
  subArmExtension extension;
  XboxController driver;
  public cmdTeleOp_ArmExtension(subArmExtension extension, XboxController driver) {
    this.extension = extension;
    this.driver = driver;
    addRequirements(extension);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    extension.teleOp(driver.getLeftY());
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
