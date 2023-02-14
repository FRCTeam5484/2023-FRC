package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.swervedrivespecialties.swervelib.MkModuleConfiguration;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class subDriveTrain extends SubsystemBase {
  public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      new Translation2d(DriveConstants.TrackWidth / 2.0, DriveConstants.WheelBase / 2.0), // Front left
      new Translation2d(DriveConstants.TrackWidth / 2.0, -DriveConstants.WheelBase / 2.0), // Front right
      new Translation2d(-DriveConstants.TrackWidth / 2.0, DriveConstants.WheelBase / 2.0), // Back left
      new Translation2d(-DriveConstants.TrackWidth / 2.0, -DriveConstants.WheelBase / 2.0)); // Back right
  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  private final SwerveDriveOdometry m_odometry;
  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte)200);

  private final SlewRateLimiter filter_vx;
  private final SlewRateLimiter filter_vy;
  private final SlewRateLimiter filter_or;

  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;

  public final SwerveDrivePoseEstimator m_poseEstimator;

  public subDriveTrain() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    MkModuleConfiguration moduleConfig = MkModuleConfiguration.getDefaultSteerFalcon500();
    moduleConfig.setDriveCurrentLimit(40.0);
    moduleConfig.setSteerCurrentLimit(30.0);

    frontLeftModule = new MkSwerveModuleBuilder(moduleConfig)
      .withLayout(tab.getLayout("Front Left Module", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(0, 0))
      .withGearRatio(SdsModuleConfigurations.MK3_STANDARD)
      .withDriveMotor(MotorType.NEO, DriveConstants.FrontLeft.DriveMotorPort)
      .withSteerMotor(MotorType.NEO, DriveConstants.FrontLeft.TurningMotorPort)
      .withSteerEncoderPort(DriveConstants.FrontLeft.TurnAbsoluteEncoderPort)
      .withSteerOffset(DriveConstants.FrontLeft.TurnAbsoluteEncoderOffsetRad)
      .build();

    frontRightModule = new MkSwerveModuleBuilder(moduleConfig)
      .withLayout(tab.getLayout("Front Right Module", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(2, 0))
      .withGearRatio(SdsModuleConfigurations.MK3_STANDARD)
      .withDriveMotor(MotorType.NEO, DriveConstants.FrontRight.DriveMotorPort)
      .withSteerMotor(MotorType.NEO, DriveConstants.FrontRight.TurningMotorPort)
      .withSteerEncoderPort(DriveConstants.FrontRight.TurnAbsoluteEncoderPort)
      .withSteerOffset(DriveConstants.FrontRight.TurnAbsoluteEncoderOffsetRad)
      .build();

    backLeftModule = new MkSwerveModuleBuilder(moduleConfig)
      .withLayout(tab.getLayout("Back Left Module", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(4, 0))
      .withGearRatio(SdsModuleConfigurations.MK3_STANDARD)
      .withDriveMotor(MotorType.NEO, DriveConstants.BackLeft.DriveMotorPort)
      .withSteerMotor(MotorType.NEO, DriveConstants.BackLeft.TurningMotorPort)
      .withSteerEncoderPort(DriveConstants.BackLeft.TurnAbsoluteEncoderPort)
      .withSteerOffset(DriveConstants.BackLeft.TurnAbsoluteEncoderOffsetRad)
      .build();

    backRightModule = new MkSwerveModuleBuilder(moduleConfig)
      .withLayout(tab.getLayout("Back Right Module", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(6, 0))
      .withGearRatio(SdsModuleConfigurations.MK3_STANDARD)
      .withDriveMotor(MotorType.NEO, DriveConstants.BackRight.DriveMotorPort)
      .withSteerMotor(MotorType.NEO, DriveConstants.BackRight.TurningMotorPort)
      .withSteerEncoderPort(DriveConstants.BackRight.TurnAbsoluteEncoderPort)
      .withSteerOffset(DriveConstants.BackRight.TurnAbsoluteEncoderOffsetRad)
      .build();

    filter_vx = new SlewRateLimiter(DriveConstants.SlewRateLimitTranslation);
    filter_vy = new SlewRateLimiter(DriveConstants.SlewRateLimitTranslation);
    filter_or = new SlewRateLimiter(DriveConstants.SlewRateLimitRotation);

    /* new Thread(() -> {
      try {
          Thread.sleep(2000);
          zeroGyroscope();
      } 
      catch (Exception e) {
      }
    }).start(); */
    zeroGyroscope();
    m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation(), getPositions(), new Pose2d());
    m_poseEstimator = new SwerveDrivePoseEstimator(m_kinematics, getGyroscopeRotation(), getPositions(), new Pose2d());
  }

  public void zeroGyroscope() {
    m_navx.reset();
    m_navx.calibrate();
  }

  public void resetGyroAt(double yaw){
    m_navx.setAngleAdjustment(yaw);
  }

  public AHRS getGyro(){
    return m_navx;
  }

  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(getGyroscopeRotation(), getPositions(), pose);
  }

  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    m_poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
  }

  public Pose2d getPosition() {
      return m_poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getGyroscopeRotation() {
    if (m_navx.isMagnetometerCalibrated()) {
      return Rotation2d.fromDegrees(m_navx.getFusedHeading());
    }
    return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds speedsModified = new ChassisSpeeds(
        filter_vx.calculate(chassisSpeeds.vxMetersPerSecond),
        filter_vy.calculate(chassisSpeeds.vyMetersPerSecond),
        filter_or.calculate(chassisSpeeds.omegaRadiansPerSecond)
    );
    driveRaw(speedsModified);
  }

  public void driveRaw(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  public void stop() {
    driveRaw(new ChassisSpeeds());
  }

  public SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {
        frontLeftModule.getPosition(),
        frontRightModule.getPosition(),
        backLeftModule.getPosition(),
        backRightModule.getPosition()
    };
  }

  @Override
  public void periodic() {
        m_poseEstimator.update(getGyroscopeRotation(), getPositions());
        m_odometry.update(getGyroscopeRotation(), getPositions());
        
        final double zeroDeadzone = 0.001;

        if (Math.abs(m_chassisSpeeds.vxMetersPerSecond) < zeroDeadzone) {
            m_chassisSpeeds.vxMetersPerSecond = 0;
        }
        if (Math.abs(m_chassisSpeeds.vyMetersPerSecond) < zeroDeadzone) {
            m_chassisSpeeds.vyMetersPerSecond = 0;
        }

        if (m_chassisSpeeds.vxMetersPerSecond == 0 && 
            m_chassisSpeeds.vyMetersPerSecond == 0 && 
            Math.abs(m_chassisSpeeds.omegaRadiansPerSecond) < zeroDeadzone) {
            m_chassisSpeeds.omegaRadiansPerSecond = 0.00001;
        }

        SmartDashboard.putNumber("DT X spd", m_chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("DT Y spd", m_chassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("DT O rot", m_chassisSpeeds.omegaRadiansPerSecond);

        SmartDashboard.putNumber("Front Left Encoder", frontLeftModule.getSteerEncoder().getAbsoluteAngle());
        SmartDashboard.putNumber("Front Right Encoder", frontRightModule.getSteerEncoder().getAbsoluteAngle());
        SmartDashboard.putNumber("Back Left Encoder", backLeftModule.getSteerEncoder().getAbsoluteAngle());
        SmartDashboard.putNumber("Back Right Encoder", backRightModule.getSteerEncoder().getAbsoluteAngle());

        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MaxVelocityMetersPerSecond);

        SwerveModulePosition[] positions = getPositions();
        for (int i = 0; i < states.length; i++) {
            states[i] = SwerveModuleState.optimize(states[i], positions[i].angle);
        }

        double flVoltage;
        double frVoltage;
        double blVoltage;
        double brVoltage;

        flVoltage = states[0].speedMetersPerSecond;
        frVoltage = states[1].speedMetersPerSecond;
        blVoltage = states[2].speedMetersPerSecond;
        brVoltage = states[3].speedMetersPerSecond;

        flVoltage = flVoltage / DriveConstants.MaxVelocityMetersPerSecond * DriveConstants.MaxVoltage;
        frVoltage = frVoltage / DriveConstants.MaxVelocityMetersPerSecond * DriveConstants.MaxVoltage;
        blVoltage = blVoltage / DriveConstants.MaxVelocityMetersPerSecond * DriveConstants.MaxVoltage;
        brVoltage = brVoltage / DriveConstants.MaxVelocityMetersPerSecond * DriveConstants.MaxVoltage;

        frontLeftModule.set(flVoltage, states[0].angle.getRadians());
        frontRightModule.set(frVoltage, states[1].angle.getRadians());
        backLeftModule.set(blVoltage, states[2].angle.getRadians());
        backRightModule.set(brVoltage, states[3].angle.getRadians());
  }
}
