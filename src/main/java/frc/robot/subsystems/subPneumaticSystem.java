package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticConstants;

public class subPneumaticSystem extends SubsystemBase {
  private PneumaticHub ph = new PneumaticHub(PneumaticConstants.PneumaticHubId);
  private Solenoid intakeSolenoid = ph.makeSolenoid(PneumaticConstants.IntakeSolenoid);

  public subPneumaticSystem() {
    SmartDashboard.setDefaultBoolean("Enable Compressor", false);
    SmartDashboard.setDefaultBoolean("Disable Compressor", false);
    ph.enableCompressorAnalog(100,120);
    close();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pressure", ph.getPressure(0));
    SmartDashboard.putBoolean("Compressor Running", ph.getCompressor());
    SmartDashboard.putBoolean("Solenoid Status", intakeSolenoid.get());

    if (SmartDashboard.getBoolean("Enable Compressor", false)) {
      SmartDashboard.putBoolean("Enable Compressor", false);
      ph.enableCompressorAnalog(SmartDashboard.getNumber("Minimum Pressure (PSI)", 0.0),SmartDashboard.getNumber("Maximum Pressure (PSI)", 0.0));
    }
    if (SmartDashboard.getBoolean("Disable Compressor", false)) {
      SmartDashboard.putBoolean("Disable Compressor", false);
      ph.disableCompressor();
    }
  }

  public void toggle(){intakeSolenoid.toggle();}
  public void open(){ intakeSolenoid.set(true);}
  public void close(){ intakeSolenoid.set(false);}
}
