package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmAngleConstants;

public class subArmAngle extends SubsystemBase {
  private final CANSparkMax angleMotor = new CANSparkMax(ArmAngleConstants.Port, MotorType.kBrushless);
  private final AbsoluteEncoder angleEncoder = angleMotor.getAbsoluteEncoder(Type.kDutyCycle);
  private final SparkMaxPIDController anglePID = angleMotor.getPIDController();

  public subArmAngle() {
    angleMotor.restoreFactoryDefaults();
    angleMotor.setInverted(ArmAngleConstants.Reversed);
    angleMotor.setIdleMode(ArmAngleConstants.Mode);
    angleMotor.setSmartCurrentLimit(ArmAngleConstants.PowerLimit);
    angleMotor.burnFlash();

    anglePID.setFeedbackDevice(angleEncoder);
    anglePID.setP(0.1);
    anglePID.setI(1e-4);
    anglePID.setD(1);
    anglePID.setIZone(0);
    anglePID.setFF(0);
    anglePID.setOutputRange(-1, 1);
  }

  @Override
  public void periodic() {
  }

  public void moveToSetPoint(double setPoint){
    anglePID.setReference(setPoint, CANSparkMax.ControlType.kPosition);
  }

  public void teleOp(double value){
    if(value >= 0 && angleEncoder.getPosition() >= ArmAngleConstants.limitPositionHigh || value <= 0 && angleEncoder.getPosition() <= ArmAngleConstants.limitPositionLow)
    {
      stop();
    }
    else{
      angleMotor.set(value);
    }
  }

  public void stop(){
    angleMotor.stopMotor();
  }
}