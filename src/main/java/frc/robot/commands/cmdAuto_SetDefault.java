package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmAngleConstants;
import frc.robot.Constants.ArmExtensionConstants;
import frc.robot.subsystems.subArmAngle;
import frc.robot.subsystems.subArmExtension;

public class cmdAuto_SetDefault extends CommandBase {
  subArmAngle angle;
  subArmExtension extension;
  PIDController anglePID = new PIDController(0.03, 0, 0);
  PIDController extensionPID = new PIDController(0.03, 0, 0);
  Timer time;
  public cmdAuto_SetDefault(subArmAngle angle, subArmExtension extension) {
    this.angle = angle;
    this.extension = extension;
    time = new Timer();
    addRequirements(angle, extension);
  }
  
  @Override
  public void initialize() {
    anglePID.reset();
    extensionPID.reset();
    time.reset();
    time.start();
  }

  @Override
  public void execute() {
    angle.angleMotor.set(anglePID.calculate(angle.getEncoderPosition(), ArmAngleConstants.DefaultPosition));
    extension.extensionMotor.set(extensionPID.calculate(extension.getEncoderPosition(), ArmExtensionConstants.DefaultPosition));
  }

  @Override
  public void end(boolean interrupted) {
    angle.stop();
    extension.stop();
  }

  @Override
  public boolean isFinished() {
    return anglePID.atSetpoint() && extensionPID.atSetpoint() || time.get() > 2 ? true : false;
  }
}
