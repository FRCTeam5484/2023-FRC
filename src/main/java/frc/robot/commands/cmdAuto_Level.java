package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.subSwerve;

public class cmdAuto_Level extends CommandBase {
  subSwerve swerve;
  PIDController levelPID = new PIDController(0.01, 0, 0);
  Timer time;
  public cmdAuto_Level(subSwerve swerve) {
    this.swerve = swerve;
    time = new Timer();
    addRequirements(swerve);
    
  }

  @Override
  public void initialize() {
    levelPID.setTolerance(3);
    levelPID.enableContinuousInput(-45, 45);
    time.reset();
    time.start();
    levelPID.reset();
  }

  @Override
  public void execute() {
    swerve.drive(levelPID.calculate(swerve.getRoll(), 0), 0, 0);
  }

  @Override
  public void end(boolean interrupted) {
    time.stop();
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return levelPID.atSetpoint() && time.hasElapsed(5) ? true : false;
  }
}