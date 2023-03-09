package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.subSwerve;

public class cmdAuto_Level extends CommandBase {
  static final double kOffBalanceAngleThresholdDegrees = 10;
  static final double kOonBalanceAngleThresholdDegrees  = 5;
  subSwerve swerve;
  PIDController levelPID = new PIDController(0.01, 0, 0);
  public cmdAuto_Level(subSwerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    levelPID.reset();
  }

  @Override
  public void execute() {
    System.out.println("Roll: " + swerve.getRoll() + " Pitch: " + swerve.getPitch() + " PID Cal: " + levelPID.calculate(swerve.getRoll(), 0));
    //swerve.drive(clamp(levelPID.calculate(swerve.getRoll(), 0), 0.2, -0.2), 0, 0);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
    //return levelPID.calculate(swerve.getRoll(), 0) < 0.01 ? true : false;
  }

  public double clamp(double source, double max, double min){
    return Math.min(Math.max(source, min), max);
  }
}