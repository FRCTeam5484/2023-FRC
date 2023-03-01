package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class subClaw extends SubsystemBase {
  private final CANSparkMax clawMotor = new CANSparkMax(ClawConstants.Port, MotorType.kBrushless);
  private final RelativeEncoder clawEncoder = clawMotor.getEncoder();
  
  public subClaw() {
    clawMotor.restoreFactoryDefaults();
    clawMotor.setInverted(ClawConstants.Reversed);
    clawMotor.setIdleMode(ClawConstants.Mode);
    clawMotor.setSmartCurrentLimit(ClawConstants.PowerLimit);
    clawMotor.burnFlash();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Claw Encoder", getEncoderPosition());
    SmartDashboard.putNumber("Claw Power", clawMotor.get());  
  }

  public void openClaw(boolean override){ 
    //System.out.println("Open: Encoder:" + clawEncoder.getPosition() + " Power:" + ClawConstants.PowerFactor);
    if(override){ clawMotor.set(ClawConstants.PowerFactor); }
    else{ clawMotor.set(clawEncoder.getPosition() <= ClawConstants.openLimit ? ClawConstants.PowerFactor : 0); }
  }
  public void closeClaw(boolean override){ 
    //System.out.println("Close: Encoder:" + clawEncoder.getPosition() + " Power:" + -ClawConstants.PowerFactor);
    if(override){ clawMotor.set(-ClawConstants.PowerFactor); }
    else{ clawMotor.set(clawEncoder.getPosition() >= ClawConstants.closeLimit ? -ClawConstants.PowerFactor : 0); } 
  }
  public void stop(){ clawMotor.stopMotor(); }
  public void resetPosition(){ clawEncoder.setPosition(0); }
  public double getEncoderPosition(){ return clawEncoder.getPosition(); }
}
