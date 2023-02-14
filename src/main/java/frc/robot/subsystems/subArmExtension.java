package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmExtensionConstants;

public class subArmExtension extends SubsystemBase {
  private final CANSparkMax extensionMotor = new CANSparkMax(ArmExtensionConstants.Port, MotorType.kBrushless);
  private final RelativeEncoder extensionEncoder = extensionMotor.getEncoder();
  private final SparkMaxPIDController extensionPID = extensionMotor.getPIDController();

  public subArmExtension() {
    extensionMotor.restoreFactoryDefaults();
    extensionMotor.setInverted(ArmExtensionConstants.Reversed);
    extensionMotor.setIdleMode(ArmExtensionConstants.Mode);
    extensionMotor.setSmartCurrentLimit(ArmExtensionConstants.PowerLimit);
    extensionMotor.burnFlash();

    extensionPID.setP(0.1);
    extensionPID.setI(1e-4);
    extensionPID.setD(1);
    extensionPID.setIZone(0);
    extensionPID.setFF(0);
    extensionPID.setOutputRange(-1, 1);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Ext Encoder", getEncoderPosition());
    SmartDashboard.putNumber("Arm Ext Power", extensionMotor.get());
  }

  public void moveToSetPoint(double setPoint){
    extensionPID.setReference(setPoint, CANSparkMax.ControlType.kPosition);
  }

  public void teleOp(double value){
    if(value > 0.05 && extensionEncoder.getPosition() <= ArmExtensionConstants.limitOpen || value < -0.05 && extensionEncoder.getPosition() >= ArmExtensionConstants.limitClosed)
    {
      extensionMotor.set(value);
    }
    else{
      stop();
    }
  }

  public void stop(){
    extensionMotor.stopMotor();
  }

  public double getEncoderPosition(){
    return extensionEncoder.getPosition();
  }

  public void resetPosition(){
    extensionEncoder.setPosition(0);
  }
  public void enablePID(double setPoint){
    extensionPID.setReference(setPoint, CANSparkMax.ControlType.kPosition);
  }
}
