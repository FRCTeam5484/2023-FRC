package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmAngleConstants;

public class subArmAngle extends SubsystemBase {
  public final CANSparkMax angleMotor = new CANSparkMax(ArmAngleConstants.Port, MotorType.kBrushless);
  private final DutyCycleEncoder throughBore = new DutyCycleEncoder(0);

  public subArmAngle() {
    angleMotor.restoreFactoryDefaults();
    angleMotor.setInverted(ArmAngleConstants.Reversed);
    angleMotor.setIdleMode(ArmAngleConstants.Mode);
    angleMotor.setSmartCurrentLimit(ArmAngleConstants.PowerLimit);
    angleMotor.burnFlash();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Ang Power", angleMotor.get());
    SmartDashboard.putNumber("Arm Ang Output", angleMotor.getAppliedOutput());
    SmartDashboard.putNumber("Arm Absolute", getEncoderPosition());
  }

  public void teleOp(double value, boolean override){
    if(override) { angleMotor.set(value); }
    else{ angleMotor.set(value <= 0 && getEncoderPosition() >= ArmAngleConstants.limitPositionLow || value >= 0 && getEncoderPosition() <= ArmAngleConstants.limitPositionHigh ? value : 0); }
  }
  public void stop(){ angleMotor.stopMotor(); }
  public double getEncoderPosition(){ return Math.abs(throughBore.getAbsolutePosition() -1)*360; }
}