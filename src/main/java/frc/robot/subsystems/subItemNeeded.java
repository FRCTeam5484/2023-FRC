package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class subItemNeeded extends SubsystemBase {
  Servo itemNeededServo;
  public subItemNeeded() {
    itemNeededServo = new Servo(0);
  }

  @Override
  public void periodic() {  }
  public void setSelection(double value){ itemNeededServo.setAngle(value); }
}
