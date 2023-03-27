package frc.robot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  UsbCamera intakeCamera;

  @Override
  public void robotInit() {
    intakeCamera = CameraServer.startAutomaticCapture();
    intakeCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    try{SmartDashboard.putString("Arm Angle Command", m_robotContainer.armAngle.getCurrentCommand().getName());} catch(Exception ex){SmartDashboard.putString("Arm Angle Command", "No Command");}
    try{SmartDashboard.putString("Arm Extension Command", m_robotContainer.armExtension.getCurrentCommand().getName());} catch(Exception ex){SmartDashboard.putString("Arm Extension Command", "No Command");}
    //try{SmartDashboard.putString("Claw Command", m_robotContainer.claw.getCurrentCommand().getName());} catch(Exception ex){SmartDashboard.putString("Claw Command", "No Command");}
    try{SmartDashboard.putString("Swerve Command", m_robotContainer.swerve.getCurrentCommand().getName());} catch(Exception ex){SmartDashboard.putString("Swerve Command", "No Command");}
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.swerve.BypassAntiTip = true;
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_robotContainer.swerve.BypassAntiTip = false;
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
