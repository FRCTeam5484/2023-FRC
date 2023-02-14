package frc.robot.classes;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule extends SubsystemBase {
  public PIDController testTurnController;
  private static final double turnkP = 0.5;
  private static final double turnkD = 0.0;
  private static final double drivekP = 0.015;

  private final CANSparkMax driveMotor;
  private final CANSparkMax turnMotor;

  public CANSparkMax getDriveMotor() {
    return driveMotor;
  }

  public CANSparkMax getTurnMotor() {
    return turnMotor;
  }

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnEncoder;

  private final CANCoder canCoder;

  private final Rotation2d offset;

  private final SparkMaxPIDController turnController;
  private final SparkMaxPIDController driveController;

  public SwerveModule(
      int driveMotorId,
      int turnMotorId,
      int canCoderId,
      double measuredOffsetRadians) {

    driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();

    turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);
    turnEncoder = turnMotor.getEncoder();
    turnController = turnMotor.getPIDController();

    testTurnController = new PIDController(0.5, 0, 0.0);
    testTurnController.enableContinuousInput(-Math.PI, Math.PI);

    canCoder = new CANCoder(canCoderId);
    canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

    offset = new Rotation2d(measuredOffsetRadians);    

    setupDrive();
    setupTurn();
  }

  public void setupDrive() {
    driveMotor.restoreFactoryDefaults();
    driveMotor.setIdleMode(IdleMode.kBrake);
    driveController.setP(drivekP);
    driveEncoder.setPositionConversionFactor(2.0 * Math.PI / SdsModuleConfigurations.MK3_STANDARD.getDriveReduction());
    driveEncoder.setVelocityConversionFactor(2.0 * Math.PI / 60 / SdsModuleConfigurations.MK3_STANDARD.getDriveReduction());
  }

  public void setupTurn() {
    turnMotor.restoreFactoryDefaults();
    turnMotor.setIdleMode(IdleMode.kBrake);
    turnController.setP(turnkP);
    turnController.setD(turnkD);
    turnEncoder.setPositionConversionFactor(2 * Math.PI / SdsModuleConfigurations.MK3_STANDARD.getSteerReduction());
  }

  public void resetDistance() { driveEncoder.setPosition(0.0); }

  public double getDriveDistanceRadians() { return driveEncoder.getPosition(); }

  public Rotation2d getCanCoderAngle() {
    double unsignedAngle = (Units.degreesToRadians(canCoder.getAbsolutePosition()) - offset.getRadians()) % (2 * Math.PI);
    return new Rotation2d(unsignedAngle);
  }

  public Rotation2d getIntegratedAngle() {
    double unsignedAngle = turnEncoder.getPosition() % (2 * Math.PI);
    if (unsignedAngle < 0) {
      unsignedAngle += 2 * Math.PI;
    }
    return new Rotation2d(unsignedAngle);
  }

  public double getCurrentVelocityRadiansPerSecond() { return driveEncoder.getVelocity(); }

  public double getCurrentVelocityMetersPerSecond() { return driveEncoder.getVelocity() * (SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() / 2.0); }

  public double getCurrentDistanceMetersPerSecond() { return driveEncoder.getPosition() * (SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() / 2.0); }

  public static double placeInAppropriate0To360Scope(double unwrappedAngle) {
    double modAngle = unwrappedAngle % (2.0 * Math.PI);
    if (modAngle < 0.0) {
      modAngle += 2.0 * Math.PI;
    }
    double wrappedAngle = modAngle;
    return wrappedAngle;
  }

  public void initTurnOffset() { turnEncoder.setPosition(getCanCoderAngle().getRadians()); }

  public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle = placeInAppropriate0To360Scope(desiredState.angle.getRadians());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = (targetAngle - currentAngle.getRadians());
    if (Math.abs(delta) > (Math.PI / 2)) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > Math.PI / 2 ? (targetAngle -= Math.PI) : (targetAngle += Math.PI);
    }
    return new SwerveModuleState(targetSpeed, new Rotation2d(targetAngle));
  }

  public void setDesiredStateClosedLoop(SwerveModuleState unoptimizedDesiredState) {
    if (Math.abs(unoptimizedDesiredState.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    SwerveModuleState optimizedDesiredState = optimize(unoptimizedDesiredState, getIntegratedAngle());
    double angularSetPoint = placeInAppropriate0To360Scope(optimizedDesiredState.angle.getRadians());
    turnMotor.set(testTurnController.calculate(getIntegratedAngle().getRadians(), angularSetPoint));
    double angularVelolictySetpoint = optimizedDesiredState.speedMetersPerSecond / (SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() / 2.0);
    if (RobotState.isAutonomous()) {
      driveMotor.setVoltage(SwerveConstants.driveFF.calculate(angularVelolictySetpoint));
    } else {
      driveMotor.set(optimizedDesiredState.speedMetersPerSecond / SwerveConstants.MaxSpeed);
    }
  }

  public void setDesiredStateClosedLoop(SwerveModuleState unoptimizedDesiredState, boolean isAutoBalancing) {
    if (Math.abs(unoptimizedDesiredState.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    SwerveModuleState optimizedDesiredState = optimize(unoptimizedDesiredState, getIntegratedAngle());
    double angularSetPoint = placeInAppropriate0To360Scope(optimizedDesiredState.angle.getRadians());
    turnMotor.set(testTurnController.calculate(getIntegratedAngle().getRadians(), angularSetPoint));
    double angularVelolictySetpoint = optimizedDesiredState.speedMetersPerSecond / (SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() / 2.0);
    if (RobotState.isAutonomous() || isAutoBalancing == true) {
      driveMotor.setVoltage(SwerveConstants.driveFF.calculate(angularVelolictySetpoint));
    } else {
      driveMotor.set(optimizedDesiredState.speedMetersPerSecond / SwerveConstants.MaxSpeed);
    }
  }

  public void resetEncoders() {
    driveEncoder.setPosition(0);
    turnEncoder.setPosition(0);
  }

  public void stop() {
    driveMotor.set(0);
    turnMotor.set(0);
  }

  public void setDriveCurrentLimit(int value){ driveMotor.setSmartCurrentLimit(value); }
  public void setTurnCurrentLimit(int value){ turnMotor.setSmartCurrentLimit(value); }
  public void setDriveIdleMode(IdleMode value){ driveMotor.setIdleMode(value); }
  public void setTurnIdleMode(IdleMode value){ turnMotor.setIdleMode(value); }
  public void burnToController(){ driveMotor.burnFlash(); turnMotor.burnFlash(); }
}
