package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticConstants;

public class subPneumaticSystem extends SubsystemBase {
  private PneumaticHub ph = new PneumaticHub(PneumaticConstants.PneumaticHubId);
  private Solenoid intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, PneumaticConstants.IntakeSolenoid);
  public subPneumaticSystem() {
    close();
  }

  @Override
  public void periodic() {
  
  }

  public void toggle(){intakeSolenoid.toggle();}
  public void open(){ intakeSolenoid.set(true);}
  public void close(){ intakeSolenoid.set(false);}
}
