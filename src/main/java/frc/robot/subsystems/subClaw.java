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

    clawPID.setP(0.1);
    clawPID.setI(1e-4);
    clawPID.setD(1);
    clawPID.setIZone(0);
    clawPID.setFF(0);
    clawPID.setOutputRange(-1, 1);
  }

  @Override
  public void periodic() {
  
  }

  public void moveToSetPoint(double setPoint){
    clawPID.setReference(setPoint, CANSparkMax.ControlType.kPosition);
  }

  public void openClaw(){
    if(clawEncoder.getPosition() >= ClawConstants.openLimit)
    {
      stop();
    }
    else{
      clawMotor.set(1);
    }
  }
  public void closeClaw(){
    if(clawEncoder.getPosition() <= ClawConstants.closeLimit)
    {
      stop();
    }
    else{
      clawMotor.set(-1);
    }
  }

  public void stop(){
    clawMotor.stopMotor();
  }
}
