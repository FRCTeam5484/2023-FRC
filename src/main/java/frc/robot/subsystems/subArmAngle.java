package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmAngleConstants;

public class subArmAngle extends SubsystemBase {
  public final CANSparkMax angleMotor = new CANSparkMax(ArmAngleConstants.Port, MotorType.kBrushless);
  private final SparkMaxPIDController anglePID = angleMotor.getPIDController();
  private final DutyCycleEncoder throughBore = new DutyCycleEncoder(0);

  public subArmAngle() {
    angleMotor.restoreFactoryDefaults();
    angleMotor.setInverted(ArmAngleConstants.Reversed);
    angleMotor.setIdleMode(ArmAngleConstants.Mode);
    angleMotor.setSmartCurrentLimit(ArmAngleConstants.PowerLimit);
    angleMotor.burnFlash();

    //angleEncoder.setPositionConversionFactor(100);

    //anglePID.setFeedbackDevice(angleEncoder);
    anglePID.setP(1);//0.1
    anglePID.setI(0);//1e-4
    anglePID.setD(0);//1
    anglePID.setIZone(0);
    anglePID.setFF(0);
    anglePID.setOutputRange(-1, 1);
  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Arm Ang Encoder", getEncoderPosition());
    SmartDashboard.putNumber("Arm Ang Power", angleMotor.get());
    SmartDashboard.putNumber("Arm Ang Output", angleMotor.getAppliedOutput());
    SmartDashboard.putNumber("Arm Absolute", getEncoderPosition());
  }

  public void moveToSetPoint(double setPoint){
    anglePID.setReference(setPoint, CANSparkMax.ControlType.kPosition);
  }

  public void teleOp(double value){
    if(value <= 0 && getEncoderPosition() >= ArmAngleConstants.limitPositionLow || value >= 0 && getEncoderPosition() <= ArmAngleConstants.limitPositionHigh)
    {
      angleMotor.set(value);
    }
    else{
      stop();
    }
  }

  public void stop(){
    angleMotor.stopMotor();
  }

  public double getEncoderPosition(){
    return Math.abs(throughBore.getAbsolutePosition() -1)*360;
    //return angleEncoder.getPosition();
  }

  public void enablePID(double setPoint){
    anglePID.setReference(setPoint, CANSparkMax.ControlType.kPosition);
  }
}