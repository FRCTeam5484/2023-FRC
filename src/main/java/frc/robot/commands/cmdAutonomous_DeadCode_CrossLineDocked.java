package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.subSwerve;

public class cmdAutonomous_DeadCode_CrossLineDocked extends CommandBase {
  subSwerve swerve;
  double speed;
  Timer time;
  public cmdAutonomous_DeadCode_CrossLineDocked(subSwerve swerve, double speed) {
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
    if(time.get() < 4){
      swerve.drive(speed, 0, 0, false);
    }
    else if (time.get() < 4.5){
      swerve.drive(0, 0, 0, false);
    }
    else if (time.get() < 6){
      swerve.drive(-speed, 0, 0, false);
    }
    else{
      swerve.drive(0, 0, 0, false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
    time.stop();
  }

  @Override
  public boolean isFinished() {
    return time.hasElapsed(6);
  }
}