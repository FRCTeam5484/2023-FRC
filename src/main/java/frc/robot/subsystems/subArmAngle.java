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
  }

  @Override
  public void periodic() {
  }
}
