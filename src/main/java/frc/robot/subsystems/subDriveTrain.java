package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.classes.SwerveModule;

public class subDriveTrain extends SubsystemBase {
  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;
  private final AHRS navX;
  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(SwerveConstants.Kinematics, new Rotation2d(), getModulePositions());
  public SwerveDriveOdometry getOdometry() { return odometry; }

  public subDriveTrain() {
    navX = new AHRS(SPI.Port.kMXP);
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        navX.reset();
        navX.calibrate();
        odometry.resetPosition(new Rotation2d(), getModulePositions(), new Pose2d());
      } catch (Exception e) {
      }
    }).start();

    frontLeftModule = new SwerveModule(
    SwerveConstants.FrontLeft.DriveMotorPort,
    SwerveConstants.FrontLeft.TurningMotorPort,
    SwerveConstants.FrontLeft.TurnAbsoluteEncoderPort,
    SwerveConstants.FrontLeft.TurnAbsoluteEncoderOffsetRad);
    SetupFrontLeftModule();

    frontRightModule = new SwerveModule(
    SwerveConstants.FrontRight.DriveMotorPort,
    SwerveConstants.FrontRight.TurningMotorPort,
    SwerveConstants.FrontRight.TurnAbsoluteEncoderPort,
    SwerveConstants.FrontRight.TurnAbsoluteEncoderOffsetRad);
    SetupFrontRightModule();

    backLeftModule = new SwerveModule(
    SwerveConstants.BackLeft.DriveMotorPort,
    SwerveConstants.BackLeft.TurningMotorPort,
    SwerveConstants.BackLeft.TurnAbsoluteEncoderPort,
    SwerveConstants.BackLeft.TurnAbsoluteEncoderOffsetRad);
    SetupBackLeftModule();

    backRightModule = new SwerveModule(
    SwerveConstants.BackRight.DriveMotorPort,
    SwerveConstants.BackRight.TurningMotorPort,
    SwerveConstants.BackRight.TurnAbsoluteEncoderPort,
    SwerveConstants.BackRight.TurnAbsoluteEncoderOffsetRad);
    SetupBackRightModule();
  }


  private void SetupFrontLeftModule() {
    frontLeftModule.initTurnOffset();
    frontLeftModule.resetDistance();
    frontLeftModule.getDriveMotor().setInverted(SwerveConstants.FrontLeft.DriveMotorReversed);
    frontLeftModule.getTurnMotor().setInverted(SwerveConstants.FrontLeft.TurningMotorReversed);
    frontLeftModule.setDriveIdleMode(SwerveConstants.FrontLeft.DriveIdleMode);
    frontLeftModule.setTurnIdleMode(SwerveConstants.FrontLeft.TurnIdleMode);
    frontLeftModule.setDriveCurrentLimit(SwerveConstants.FrontLeft.DriveCurrentLimit);
    frontLeftModule.setTurnCurrentLimit(SwerveConstants.FrontLeft.TurnCurrentLimit);
    frontLeftModule.burnToController();
  }

  private void SetupFrontRightModule() {
    frontRightModule.initTurnOffset();
    frontRightModule.resetDistance();
    frontRightModule.getDriveMotor().setInverted(SwerveConstants.FrontRight.DriveMotorReversed);
    frontRightModule.getTurnMotor().setInverted(SwerveConstants.FrontRight.TurningMotorReversed);
    frontRightModule.setDriveIdleMode(SwerveConstants.FrontRight.DriveIdleMode);
    frontRightModule.setTurnIdleMode(SwerveConstants.FrontRight.TurnIdleMode);
    frontRightModule.setDriveCurrentLimit(SwerveConstants.FrontRight.DriveCurrentLimit);
    frontRightModule.setTurnCurrentLimit(SwerveConstants.FrontRight.TurnCurrentLimit);
    frontRightModule.burnToController();
  }

  private void SetupBackLeftModule() {
    backLeftModule.initTurnOffset();
    backLeftModule.resetDistance();
    backLeftModule.getDriveMotor().setInverted(SwerveConstants.BackLeft.DriveMotorReversed);
    backLeftModule.getTurnMotor().setInverted(SwerveConstants.BackLeft.TurningMotorReversed);
    backLeftModule.setDriveIdleMode(SwerveConstants.BackLeft.DriveIdleMode);
    backLeftModule.setTurnIdleMode(SwerveConstants.BackLeft.TurnIdleMode);
    backLeftModule.setDriveCurrentLimit(SwerveConstants.BackLeft.DriveCurrentLimit);
    backLeftModule.setTurnCurrentLimit(SwerveConstants.BackLeft.TurnCurrentLimit);
    backLeftModule.burnToController();
  }

  private void SetupBackRightModule() {
    backRightModule.initTurnOffset();
    backRightModule.resetDistance();
    backRightModule.getDriveMotor().setInverted(SwerveConstants.BackRight.DriveMotorReversed);
    backRightModule.getTurnMotor().setInverted(SwerveConstants.BackRight.TurningMotorReversed);
    backRightModule.setDriveIdleMode(SwerveConstants.BackRight.DriveIdleMode);
    backRightModule.setTurnIdleMode(SwerveConstants.BackRight.TurnIdleMode);
    backRightModule.setDriveCurrentLimit(SwerveConstants.BackRight.DriveCurrentLimit);
    backRightModule.setTurnCurrentLimit(SwerveConstants.BackRight.TurnCurrentLimit);
    backRightModule.burnToController();
  }

  @Override
  public void periodic() {
    odometry.update(getHeading(), getModulePositions());
    SmartDashboard.putString("Robot pose", getPose().toString());
    SmartDashboard.putNumber("navX Heading", getHeading().getDegrees());
    SmartDashboard.putNumber("roll", navX.getRoll());
    SmartDashboard.putNumber("pitch", navX.getPitch());
  }

  public void drive(double forward, double strafe, double rotation, boolean isFieldRelative) {
    ChassisSpeeds speeds = isFieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, getHeading()) : new ChassisSpeeds(forward, strafe, rotation);
    SwerveModuleState[] states = SwerveConstants.Kinematics.toSwerveModuleStates(speeds);
    setModuleStates(states);
  }

  public void drive(double forward, double strafe, double rotation, boolean isFieldRelative, boolean isAutoBalancing) {
    ChassisSpeeds speeds = isFieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, getHeading()) : new ChassisSpeeds(forward, strafe, rotation);
    SwerveModuleState[] states = SwerveConstants.Kinematics.toSwerveModuleStates(speeds);
    setModuleStates(states, isAutoBalancing);
  }

  public void setModuleStates(SwerveModuleState[] moduleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.MaxSpeed);
    frontLeftModule.setDesiredStateClosedLoop(moduleStates[0]);
    frontRightModule.setDesiredStateClosedLoop(moduleStates[1]);
    backLeftModule.setDesiredStateClosedLoop(moduleStates[2]);
    backRightModule.setDesiredStateClosedLoop(moduleStates[3]);
  }

  public void setModuleStates(SwerveModuleState[] moduleStates, boolean isAutoBalancing) {
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.MaxSpeed);
    frontLeftModule.setDesiredStateClosedLoop(moduleStates[0], isAutoBalancing);
    frontRightModule.setDesiredStateClosedLoop(moduleStates[1], isAutoBalancing);
    backLeftModule.setDesiredStateClosedLoop(moduleStates[2], isAutoBalancing);
    backRightModule.setDesiredStateClosedLoop(moduleStates[3], isAutoBalancing);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = {
        new SwerveModuleState(frontLeftModule.getCurrentVelocityMetersPerSecond(), frontLeftModule.getIntegratedAngle()),
        new SwerveModuleState(frontRightModule.getCurrentVelocityMetersPerSecond(), frontRightModule.getIntegratedAngle()),
        new SwerveModuleState(backLeftModule.getCurrentVelocityMetersPerSecond(), backLeftModule.getIntegratedAngle()),
        new SwerveModuleState(backRightModule.getCurrentVelocityMetersPerSecond(), backRightModule.getIntegratedAngle())
    };
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = {
        new SwerveModulePosition(frontLeftModule.getCurrentDistanceMetersPerSecond(), frontLeftModule.getIntegratedAngle()),
        new SwerveModulePosition(frontRightModule.getCurrentDistanceMetersPerSecond(), frontRightModule.getIntegratedAngle()),
        new SwerveModulePosition(backLeftModule.getCurrentDistanceMetersPerSecond(), backLeftModule.getIntegratedAngle()),
        new SwerveModulePosition(backRightModule.getCurrentDistanceMetersPerSecond(), backRightModule.getIntegratedAngle())
    };
    return positions;
  }

  public Pose2d getPose() { return odometry.getPoseMeters(); }

  public void resetOdometry(Pose2d pose) { odometry.resetPosition(getHeading(), getModulePositions(), pose); }

  public void resetDriveDistances() {
    frontLeftModule.resetDistance();
    frontRightModule.resetDistance();
    backLeftModule.resetDistance();
    backRightModule.resetDistance();
  }

  public Rotation2d getHeading() { return Rotation2d.fromDegrees(-navX.getYaw()); }

  public AHRS getNavX() { return navX; }

  public void stopModules() {
    frontLeftModule.stop();
    frontRightModule.stop();
    backRightModule.stop();
    backLeftModule.stop();
  }

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
    PIDController xController = new PIDController(3, 0, 0);
    PIDController yController = new PIDController(3, 0, 0);
    PIDController thetaController = new PIDController(1, 0.0, 0.0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    PPSwerveControllerCommand command = new PPSwerveControllerCommand(
        trajectory,
        this::getPose,
        SwerveConstants.Kinematics,
        xController,
        yController,
        thetaController,
        this::setModuleStates,
        this);
    return command;
  }
}