package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmExtensionConstants;

public class subArmExtension extends SubsystemBase {
  public final CANSparkMax extensionMotor = new CANSparkMax(ArmExtensionConstants.Port, MotorType.kBrushless);
  private final RelativeEncoder extensionEncoder = extensionMotor.getEncoder();

  public subArmExtension() {
    extensionMotor.restoreFactoryDefaults();
    extensionMotor.setInverted(ArmExtensionConstants.Reversed);
    extensionMotor.setIdleMode(ArmExtensionConstants.Mode);
    extensionMotor.setSmartCurrentLimit(ArmExtensionConstants.PowerLimit);
    extensionMotor.burnFlash();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Ext Encoder", getEncoderPosition());
    //SmartDashboard.putNumber("Arm Ext Power", extensionMotor.get());
    //SmartDashboard.putNumber("Arm Ext Output", extensionMotor.getAppliedOutput());
  }

  public void teleOp(double value, boolean override){
    if(override) { extensionMotor.set(value); }
    else { extensionMotor.set(value > 0.05 && extensionEncoder.getPosition() <= ArmExtensionConstants.limitOpen || value < -0.05 && extensionEncoder.getPosition() >= ArmExtensionConstants.limitClosed ? value : 0); }
  }
  public void stop(){ extensionMotor.stopMotor(); }
  public double getEncoderPosition(){ return extensionEncoder.getPosition(); }
  public void resetPosition(){ extensionEncoder.setPosition(0); }
}
