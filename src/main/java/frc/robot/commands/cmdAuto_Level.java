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
    time.reset();
    time.start();
    levelPID.reset();
  }

  @Override
  public void execute() {
    System.out.println("Roll: " + swerve.getRoll() + " PID Cal: " + levelPID.calculate(swerve.getRoll(), 0));
    swerve.drive(clamp(levelPID.calculate(swerve.getRoll(), 0), 0.08, -0.08), 0, 0);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    //return time.hasElapsed(8);
    //return levelPID.atSetpoint();
    return Math.abs(levelPID.calculate(swerve.getRoll(), 0)) < 0.01 && time.hasElapsed(8) ? true : false;
  }

  public double clamp(double source, double max, double min){
    return Math.min(Math.max(source, min), max);
  }
}