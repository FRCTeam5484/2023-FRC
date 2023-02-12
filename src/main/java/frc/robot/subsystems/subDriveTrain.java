package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class subDriveTrain extends SubsystemBase {
  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      new Translation2d(DriveConstants.TrackWidth / 2.0, DriveConstants.WheelBase / 2.0), // Front left
      new Translation2d(DriveConstants.TrackWidth / 2.0, -DriveConstants.WheelBase / 2.0), // Front right
      new Translation2d(-DriveConstants.TrackWidth / 2.0, DriveConstants.WheelBase / 2.0), // Back left
      new Translation2d(-DriveConstants.TrackWidth / 2.0, -DriveConstants.WheelBase / 2.0)); // Back right
  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  //private final SwerveDriveOdometry odometer;
  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte)200);


  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;  

  public subDriveTrain() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    frontLeftModule = Mk3SwerveModuleHelper.createNeo(
      tab.getLayout("Front Left Module", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(0, 0), 
      Mk3SwerveModuleHelper.GearRatio.STANDARD,
      DriveConstants.FrontLeft.DriveMotorPort,
      DriveConstants.FrontLeft.TurningMotorPort,
      DriveConstants.FrontLeft.TurnAbsoluteEncoderPort,
      DriveConstants.FrontLeft.TurnAbsoluteEncoderOffsetRad);

    frontRightModule = Mk3SwerveModuleHelper.createNeo(
      tab.getLayout("Front Right Module", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(2, 0), 
      Mk3SwerveModuleHelper.GearRatio.STANDARD,
      DriveConstants.FrontRight.DriveMotorPort,
      DriveConstants.FrontRight.TurningMotorPort,
      DriveConstants.FrontRight.TurnAbsoluteEncoderPort,
      DriveConstants.FrontRight.TurnAbsoluteEncoderOffsetRad);

    backLeftModule = Mk3SwerveModuleHelper.createNeo(
      tab.getLayout("Back Left Module", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(4, 0), 
      Mk3SwerveModuleHelper.GearRatio.STANDARD,
      DriveConstants.BackLeft.DriveMotorPort,
      DriveConstants.BackLeft.TurningMotorPort,
      DriveConstants.BackLeft.TurnAbsoluteEncoderPort,
      DriveConstants.BackLeft.TurnAbsoluteEncoderOffsetRad);

    backRightModule = Mk3SwerveModuleHelper.createNeo(
      tab.getLayout("Back Right Module", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(6, 0), 
      Mk3SwerveModuleHelper.GearRatio.STANDARD,
      DriveConstants.BackRight.DriveMotorPort,
      DriveConstants.BackRight.TurningMotorPort,
      DriveConstants.BackRight.TurnAbsoluteEncoderPort,
      DriveConstants.BackRight.TurnAbsoluteEncoderOffsetRad);

    //odometer = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation(), new SwerveModulePosition[] {frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition()});

    new Thread(() -> {
      try {
          Thread.sleep(1000);
          zeroGyroscope();
      } 
      catch (Exception e) {
      }
    }).start();
  }

  public void zeroGyroscope() {
    m_navx.zeroYaw();
  }

  public Rotation2d getGyroscopeRotation() {
    if (m_navx.isMagnetometerCalibrated()) {
      return Rotation2d.fromDegrees(m_navx.getFusedHeading());
    }
    return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  @Override
  public void periodic() {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MaxVelocityMetersPerSecond);

    frontLeftModule.set(states[0].speedMetersPerSecond / DriveConstants.MaxVelocityMetersPerSecond * DriveConstants.MaxVoltage, states[0].angle.getRadians());
    frontRightModule.set(states[1].speedMetersPerSecond / DriveConstants.MaxVelocityMetersPerSecond * DriveConstants.MaxVoltage, states[1].angle.getRadians());
    backLeftModule.set(states[2].speedMetersPerSecond / DriveConstants.MaxVelocityMetersPerSecond * DriveConstants.MaxVoltage, states[2].angle.getRadians());
    backRightModule.set(states[3].speedMetersPerSecond / DriveConstants.MaxVelocityMetersPerSecond * DriveConstants.MaxVoltage, states[3].angle.getRadians());
  }
}
