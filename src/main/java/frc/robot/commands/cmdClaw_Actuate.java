package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.subClaw;

public class cmdClaw_Actuate extends CommandBase {
  subClaw claw;
  DoubleSupplier clawOpenCommand;
  DoubleSupplier clawCloseCommand;
  public cmdClaw_Actuate(subClaw claw, DoubleSupplier clawOpenCommand, DoubleSupplier clawCloseCommand) {
    this.claw = claw;
    this.clawOpenCommand = clawOpenCommand;
    this.clawCloseCommand = clawCloseCommand;
    addRequirements(claw);
  }

  
  @Override
  public void initialize() {}

  
  @Override
  public void execute() {
    if(clawOpenCommand.getAsDouble() > 0.8)
    {
      claw.openClaw();
    }
    else if(clawCloseCommand.getAsDouble() > 0.8)
    {
      claw.closeClaw();
    }
    else
    {
      claw.stop();
    }
  }

  
  @Override
  public void end(boolean interrupted) {
    claw.stop();
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
