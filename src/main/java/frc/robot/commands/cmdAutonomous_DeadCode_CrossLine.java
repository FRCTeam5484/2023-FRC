package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.subSwerve;

public class cmdAutonomous_DeadCode_CrossLine extends CommandBase {
  subSwerve swerve;
  double speed;
  Timer time;
  public cmdAutonomous_DeadCode_CrossLine(subSwerve swerve, double speed) {
    this.swerve = swerve;
    this.speed = speed;
    addRequirements(swerve);
    time = new Timer();
  }

  @Override
  public void initialize() {
    time.reset();
    time.start();
  }

  @Override
  public void execute() {
    swerve.drive(speed, 0, 0);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return time.hasElapsed(3);
  }
}
