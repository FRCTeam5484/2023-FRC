package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.subArmAngle;
import frc.robot.subsystems.subArmExtension;

public class cmdAuto_SetGoal extends CommandBase {
  subArmAngle angle;
  subArmExtension extension;
  double angleGoal;
  double extensionGoal;
  PIDController anglePID = new PIDController(0.1, 1e-4, 1);
  PIDController extensionPID = new PIDController(0.1, 1e-4, 1);

  public cmdAuto_SetGoal(subArmAngle angle, subArmExtension extension, double angleGoal, double extensionGoal) {
    this.angle = angle;
    this.extension = extension;
    this.angleGoal = angleGoal;
    this.extensionGoal = extensionGoal;
    addRequirements(angle, extension);
  }
  
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    System.out.println("Angle Command: " + anglePID.calculate(angle.getEncoderPosition(), angleGoal) + "  Extension Command: " + extensionPID.calculate(extension.getEncoderPosition(), extensionGoal));
    //angle.angleMotor.set(anglePID.calculate(angle.getEncoderPosition(), angleGoal));
    //extension.extensionMotor.set(extensionPID.calculate(extension.getEncoderPosition(), extensionGoal));
  }

  @Override
  public void end(boolean interrupted) {
    angle.stop();
    extension.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
