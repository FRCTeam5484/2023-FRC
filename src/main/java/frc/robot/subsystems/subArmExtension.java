package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
  }

  @Override
  public void periodic() {
  
  }
}
