package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.subArmAngle;
import frc.robot.subsystems.subArmExtension;

public class cmdAuto_SetGoalWithTimeout extends CommandBase {
  subArmAngle angle;
  subArmExtension extension;
  double angleGoal;
  double extensionGoal;
  double timeout;
  PIDController anglePID = new PIDController(0.01, 0, 0);
  PIDController extensionPID = new PIDController(0.02, 0, 0);
  Timer time = new Timer();
  
  public cmdAuto_SetGoalWithTimeout(subArmAngle angle, subArmExtension extension, double angleGoal, double extensionGoal, double timeout) {
    this.angle = angle;
    this.extension = extension;
    this.angleGoal = angleGoal;
    this.extensionGoal = extensionGoal;
    this.timeout = timeout;
    addRequirements(angle, extension);
  }
  
  @Override
  public void initialize() {
    time.reset();
    time.start();
    anglePID.reset();
    extensionPID.reset();
  }

  @Override
  public void execute() {
    //System.out.println("Angle Command: " + anglePID.calculate(angle.getEncoderPosition(), angleGoal) + "  Extension Command: " + extensionPID.calculate(extension.getEncoderPosition(), extensionGoal));
    angle.angleMotor.set(anglePID.calculate(angle.getEncoderPosition(), angleGoal));
    extension.extensionMotor.set(extensionPID.calculate(extension.getEncoderPosition(), extensionGoal));
  }

  @Override
  public void end(boolean interrupted) {
    angle.stop();
    extension.stop();
  }

  @Override
  public boolean isFinished() {
    return time.hasElapsed(timeout) ? true : false;
  }
}
