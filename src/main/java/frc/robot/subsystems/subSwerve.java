package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.classes.SwerveModule;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class subSwerve extends SubsystemBase {

  private final SwerveModule frontLeftModule = new SwerveModule(
    "Front Left", 
    SwerveConstants.FrontLeft.DriveMotorPort, 
    SwerveConstants.FrontLeft.DriveIdleMode, 
    SwerveConstants.FrontLeft.DriveCurrentLimit, 
    SwerveConstants.FrontLeft.DriveMotorReversed, 
    SwerveConstants.FrontLeft.RotationMotorPort, 
    SwerveConstants.FrontLeft.RotationIdleMode, 
    SwerveConstants.FrontLeft.RotationCurrentLimit, 
    SwerveConstants.FrontLeft.RotationMotorReversed,
    SwerveConstants.FrontLeft.RotationAbsoluteEncoderOffset);

  private final SwerveModule frontRightModule = new SwerveModule(
    "Front Right", 
    SwerveConstants.FrontRight.DriveMotorPort, 
    SwerveConstants.FrontRight.DriveIdleMode, 
    SwerveConstants.FrontRight.DriveCurrentLimit, 
    SwerveConstants.FrontRight.DriveMotorReversed, 
    SwerveConstants.FrontRight.RotationMotorPort, 
    SwerveConstants.FrontRight.RotationIdleMode, 
    SwerveConstants.FrontRight.RotationCurrentLimit, 
    SwerveConstants.FrontRight.RotationMotorReversed,
    SwerveConstants.FrontRight.RotationAbsoluteEncoderOffset);

  private final SwerveModule backLeftModule = new SwerveModule(
      "Back Left", 
      SwerveConstants.BackLeft.DriveMotorPort, 
      SwerveConstants.BackLeft.DriveIdleMode, 
      SwerveConstants.BackLeft.DriveCurrentLimit, 
      SwerveConstants.BackLeft.DriveMotorReversed, 
      SwerveConstants.BackLeft.RotationMotorPort, 
      SwerveConstants.BackLeft.RotationIdleMode, 
      SwerveConstants.BackLeft.RotationCurrentLimit, 
      SwerveConstants.BackLeft.RotationMotorReversed,
      SwerveConstants.BackLeft.RotationAbsoluteEncoderOffset);

  private final SwerveModule backRightModule = new SwerveModule(
      "Back Right", 
      SwerveConstants.BackRight.DriveMotorPort, 
      SwerveConstants.BackRight.DriveIdleMode, 
      SwerveConstants.BackRight.DriveCurrentLimit, 
      SwerveConstants.BackRight.DriveMotorReversed, 
      SwerveConstants.BackRight.RotationMotorPort, 
      SwerveConstants.BackRight.RotationIdleMode, 
      SwerveConstants.BackRight.RotationCurrentLimit, 
      SwerveConstants.BackRight.RotationMotorReversed,
      SwerveConstants.BackRight.RotationAbsoluteEncoderOffset);
  
  private final AHRS gyro;
  public SwerveDriveOdometry odometry;
  
  public subSwerve() {
    gyro = new AHRS(SPI.Port.kMXP);
    odometry = new SwerveDriveOdometry(
      SwerveConstants.SwerveKinematics,
      Rotation2d.fromDegrees(0),
      new SwerveModulePosition[] {
        frontLeftModule.getPosition(),
        frontRightModule.getPosition(),
        backLeftModule.getPosition(),
        backRightModule.getPosition()
      });
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        gyro.reset();
        gyro.calibrate();
      } catch (Exception e) { }
    }).start();
  }
  public Pose2d getPose() { return odometry.getPoseMeters(); }
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
      Rotation2d.fromDegrees(gyro.getAngle()),
      new SwerveModulePosition[] {
        frontLeftModule.getPosition(),
        frontRightModule.getPosition(),
        backLeftModule.getPosition(),
        backRightModule.getPosition()
      },
      pose);
  }
  public void updateOdometry(){
    odometry.update(
    Rotation2d.fromDegrees(gyro.getAngle()),
      new SwerveModulePosition[] {
        frontLeftModule.getPosition(),
        frontRightModule.getPosition(),
        backLeftModule.getPosition(),
        backRightModule.getPosition()
      });
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    xSpeed *= SwerveConstants.MaxSpeedMetersPerSecond;
    ySpeed *= SwerveConstants.MaxSpeedMetersPerSecond;
    rot *= SwerveConstants.MaxAngularSpeed;
    var swerveModuleStates = SwerveConstants.SwerveKinematics.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d()) : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MaxSpeedMetersPerSecond);
    frontLeftModule.setDesiredState(swerveModuleStates[0]);
    frontRightModule.setDesiredState(swerveModuleStates[1]);
    backLeftModule.setDesiredState(swerveModuleStates[2]);
    backRightModule.setDesiredState(swerveModuleStates[3]);
  }

  public void setXMode() {
    frontLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MaxSpeedMetersPerSecond);
    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    backLeftModule.setDesiredState(desiredStates[2]);
    backRightModule.setDesiredState(desiredStates[3]);
  }

  public void resetEncoders() {
    frontLeftModule.resetEncoders();
    frontRightModule.resetEncoders();
    backLeftModule.resetEncoders();
    backRightModule.resetEncoders();
  }

  public void stopModules(){
    frontLeftModule.stopModule();
    frontRightModule.stopModule();
    backLeftModule.stopModule();
    backRightModule.stopModule();
  }

  public void zeroHeading() { gyro.reset(); }
  public double getHeading() { return Rotation2d.fromDegrees(gyro.getAngle()).getDegrees(); }
  public double getRotationRate() { return gyro.getRate() * (SwerveConstants.GyroReversed ? -1.0 : 1.0); }

  public SequentialCommandGroup followPathCmd(String pathName) {
    PathPlannerTrajectory trajectory = getPathPlannerTrajectory(pathName);
    Command ppCommand = getPathPlannerCommand(trajectory);
    return new SequentialCommandGroup(
        new InstantCommand(() -> this.resetOdometry(trajectory.getInitialPose())),
        ppCommand,
        new InstantCommand(() -> this.stopModules()));
  }

  public PathPlannerTrajectory getPathPlannerTrajectory(String pathName) {
    PathConstraints constraints = PathPlanner.getConstraintsFromPath(pathName);
    PathPlannerTrajectory ppTrajectory = PathPlanner.loadPath(pathName, constraints, false);
    return ppTrajectory;
  }

  public Command getPathPlannerCommand(PathPlannerTrajectory trajectory) {
    //var thetaController = new ProfiledPIDController(SwerveConstants.Auto.PThetaController, 0, 0, SwerveConstants.Auto.ThetaControllerConstraints);
    var thetaController = new PIDController(SwerveConstants.Auto.PThetaController, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    PPSwerveControllerCommand command = new PPSwerveControllerCommand(
      trajectory,
      this::getPose,
      SwerveConstants.SwerveKinematics, 
      new PIDController(SwerveConstants.Auto.PXController, 0, 0),
      new PIDController(SwerveConstants.Auto.PYController, 0,0),
      thetaController,
      this::setModuleStates,
      this);
    return command;
  }

  public ChassisSpeeds getChassisSpeeds(){ return SwerveConstants.SwerveKinematics.toChassisSpeeds(frontLeftModule.getState(), frontRightModule.getState(), backLeftModule.getState(), backRightModule.getState());}

  @Override
  public void periodic() {
    updateOdometry();
    SmartDashboard.putNumber("Robot Speed X", getChassisSpeeds().vxMetersPerSecond);
    SmartDashboard.putNumber("Robot Speed Y", getChassisSpeeds().vyMetersPerSecond);
    SmartDashboard.putNumber("Robot Omega", getChassisSpeeds().omegaRadiansPerSecond);
    SmartDashboard.putNumber("Front Left Angle Raw", frontLeftModule.getAngleRaw());
    SmartDashboard.putNumber("Front Left Angle Calculated", frontLeftModule.GetModuleAngle());
    SmartDashboard.putNumber("Front Left Drive MPS", frontLeftModule.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Front Left Drive Power", frontLeftModule.getDrivePower());
    SmartDashboard.putNumber("Front Left Rotation Power", frontLeftModule.getRotationPower());
    SmartDashboard.putNumber("Front Right Angle Raw", frontRightModule.getAngleRaw());
    SmartDashboard.putNumber("Front Right Angle Calculated", frontRightModule.GetModuleAngle());
    SmartDashboard.putNumber("Front Right Drive MPS", frontRightModule.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Front Right Drive Power", frontRightModule.getDrivePower());
    SmartDashboard.putNumber("Front Right Rotation Power", frontRightModule.getRotationPower());
    SmartDashboard.putNumber("Back Left Angle Raw", backLeftModule.getAngleRaw());
    SmartDashboard.putNumber("Back Left Angle Calculated", backLeftModule.GetModuleAngle());
    SmartDashboard.putNumber("Back Left Drive MPS", backLeftModule.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Back Left Drive Power", backLeftModule.getDrivePower());
    SmartDashboard.putNumber("Back Left Rotation Power", backLeftModule.getRotationPower());
    SmartDashboard.putNumber("Back Right Angle Raw", backRightModule.getAngleRaw());
    SmartDashboard.putNumber("Back Right Angle Calculated", backRightModule.GetModuleAngle());
    SmartDashboard.putNumber("Back Right Drive MPS", backRightModule.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Back Right Drive Power", backRightModule.getDrivePower());
    SmartDashboard.putNumber("Back Right Rotation Power", backRightModule.getRotationPower());
    SmartDashboard.putNumber("Heading", getHeading());
    SmartDashboard.putNumber("Rotation Rate", getRotationRate());
  }
}