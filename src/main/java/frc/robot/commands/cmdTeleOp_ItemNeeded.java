package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.subItemNeeded;

public class cmdTeleOp_ItemNeeded extends CommandBase {
  subItemNeeded item;
  double angle;
  public cmdTeleOp_ItemNeeded(subItemNeeded item, double angle) {
    this.item = item;
    this.angle = angle;
    addRequirements(item);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    item.setSelection(angle);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
