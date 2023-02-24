package frc.robot.commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.subClaw;

public class cmdClaw_TeleOp extends CommandBase {
  subClaw claw;
  DoubleSupplier clawOpenCommand;
  DoubleSupplier clawCloseCommand;
  BooleanSupplier overrideSafety;
  public cmdClaw_TeleOp(subClaw claw, DoubleSupplier clawOpenCommand, DoubleSupplier clawCloseCommand, BooleanSupplier overrideSafety) {
    this.claw = claw;
    this.clawOpenCommand = clawOpenCommand;
    this.clawCloseCommand = clawCloseCommand;
    this.overrideSafety = overrideSafety;
    addRequirements(claw);
  }

  
  @Override
  public void initialize() {}

  
  @Override
  public void execute() {
    if(clawOpenCommand.getAsDouble() > 0.8)
    {
      claw.openClaw(overrideSafety.getAsBoolean());
    }
    else if(clawCloseCommand.getAsDouble() > 0.8)
    {
      claw.closeClaw(overrideSafety.getAsBoolean());
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
