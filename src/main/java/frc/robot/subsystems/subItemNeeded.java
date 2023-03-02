package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ServoConstants;

public class subItemNeeded extends SubsystemBase {
  Servo itemNeededServo;
  public enum itemList {
    ConeUp,
    ConeDown,
    CubeUp,
    CubeDown;
  }
  public subItemNeeded() {
    itemNeededServo = new Servo(ServoConstants.servoPort);
    setCurrentSelection(itemList.ConeUp);
  }

  @Override
  public void periodic() {  }
  public void setCurrentSelection(itemList selected){ 
    switch(selected){
      case ConeUp : itemNeededServo.setAngle(ServoConstants.coneUp); break;
      case ConeDown : itemNeededServo.setAngle(ServoConstants.coneDown); break;
      case CubeUp : itemNeededServo.setAngle(ServoConstants.cubeUp); break;
      case CubeDown : itemNeededServo.setAngle(ServoConstants.cubeDown); break;
    }
    SmartDashboard.putString("Human Player Item", selected.toString());
  }
}
