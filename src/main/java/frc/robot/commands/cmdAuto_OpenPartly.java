package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.subPneumatic;

public class cmdAuto_OpenPartly extends CommandBase {
  subPneumatic air;
  Timer time;
  public cmdAuto_OpenPartly(subPneumatic air) {
    this.air = air;
    time = new Timer();
    addRequirements(air);
  }

  @Override
  public void initialize() {
    time.reset();
    time.start();
  }

  @Override
  public void execute() {
    air.close();
    if(time.hasElapsed(0.2)){
      air.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return time.hasElapsed(0.5);
  }
}
