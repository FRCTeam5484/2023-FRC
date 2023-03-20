package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.subSwerve;


public class cmdAuto_TurnAround extends CommandBase {
  subSwerve swerve;
  double headingGoal;
  PIDController headingPID = new PIDController(0.015, 0, 0);
  Timer time;
  public cmdAuto_TurnAround(subSwerve swerve) {
    this.swerve = swerve;
    time = new Timer();
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    headingPID.setTolerance(10);
    headingPID.enableContinuousInput(0, 360);
    headingPID.reset();
    headingGoal = 180;
    time.reset();
    time.start();
  }

  @Override
  public void execute() {
    swerve.drive(0, 0, -headingPID.calculate(swerve.getHeading(), headingGoal), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    time.stop();
    swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return headingPID.atSetpoint() && time.hasElapsed(3) ? true : false;
  }
}
