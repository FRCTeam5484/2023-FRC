package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
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

  public static final double MAX_VOLTAGE = 12.0;
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 5676.0 / 60.0 * SdsModuleConfigurations.MK3_STANDARD.getDriveReduction() * SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI;
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(DriveConstants.TrackWidth / 2.0, DriveConstants.WheelBase / 2.0);
  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      new Translation2d(-DriveConstants.TrackWidth / 2.0, DriveConstants.WheelBase / 2.0), // Front left
      new Translation2d(-DriveConstants.TrackWidth / 2.0, -DriveConstants.WheelBase / 2.0), // Front right
      new Translation2d(DriveConstants.TrackWidth / 2.0, DriveConstants.WheelBase / 2.0), // Back left
      new Translation2d(DriveConstants.TrackWidth / 2.0, -DriveConstants.WheelBase / 2.0)); // Back right
  
  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte)200);

  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  

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
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
  }
}