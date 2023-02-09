package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class subClaw extends SubsystemBase {
  private final CANSparkMax clawMotor = new CANSparkMax(ClawConstants.Port, MotorType.kBrushless);
  private final RelativeEncoder clawEncoder = clawMotor.getEncoder();
  private final SparkMaxPIDController clawPID = clawMotor.getPIDController();
  
  public subClaw() {
    clawMotor.restoreFactoryDefaults();
    clawMotor.setInverted(ClawConstants.Reversed);
    clawMotor.setIdleMode(ClawConstants.Mode);
    clawMotor.setSmartCurrentLimit(ClawConstants.PowerLimit);
    clawMotor.burnFlash();
  }

  @Override
  public void periodic() {
  
  }
}
