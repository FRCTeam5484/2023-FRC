package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.subArmAngle;
import frc.robot.subsystems.subPneumatic;

public class cmdAuto_GripCone extends CommandBase {
  subArmAngle angle;
  subPneumatic air;
  PIDController anglePID = new PIDController(0.01, 0, 0);
  Timer time;
  public cmdAuto_GripCone(subArmAngle angle, subPneumatic air) {
    this.angle = angle;
    this.air = air;
    time = new Timer();
    addRequirements(angle, air);
  }

  @Override
  public void initialize() {
    time.reset();
    time.start();
    anglePID.reset();
  }

  @Override
  public void execute() {
    angle.angleMotor.set(anglePID.calculate(angle.getEncoderPosition(), 90));
    if(!time.hasElapsed(1))
    {
      air.close();
    }
    else if(!time.hasElapsed(1.2)){
      air.open();
    }
    else if(!time.hasElapsed(1.25)){
      air.close();
    }
  }

  @Override
  public void end(boolean interrupted) {
    angle.stop();
  }

  @Override
  public boolean isFinished() {
    return time.hasElapsed(2);
  }
}
