package frc.robot.classes;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
  private final String moduleName;
  private final CANSparkMax driveMotor;
  private final CANSparkMax rotationMotor;
  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnEncoder;
  private final CANCoder rotationEncoder;
  private final PIDController rotationPID;

  public SwerveModule(String moduleName, int drivePort, IdleMode driveIdle, int driveCurrentLimit, boolean driveReversed, int rotationPort, IdleMode rotationIdle, int rotationCurrentLimit, boolean rotationReversed, int rotationEncoderPort, double rotationEncoderOffset, boolean rotationEncoderReversed) {
    this.moduleName = moduleName;
    rotationEncoder = new CANCoder(rotationEncoderPort);
        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.magnetOffsetDegrees = rotationEncoderOffset;
        canCoderConfiguration.sensorDirection = rotationEncoderReversed;
        canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        rotationEncoder.configAllSettings(canCoderConfiguration);

    driveMotor = new CANSparkMax(drivePort, MotorType.kBrushless);
    driveMotor.restoreFactoryDefaults();
    driveMotor.setIdleMode(driveIdle);
    driveMotor.setSmartCurrentLimit(driveCurrentLimit);
    driveMotor.setInverted(driveReversed);
    driveMotor.enableVoltageCompensation(SwerveConstants.VoltCompensation);

    driveEncoder = driveMotor.getEncoder();
    driveEncoder.setPositionConversionFactor(SwerveConstants.DriveEncoderPositionFactor);
    driveEncoder.setVelocityConversionFactor(SwerveConstants.DriveEncoderVelocityFactor);
    driveMotor.burnFlash();

    rotationMotor = new CANSparkMax(rotationPort, MotorType.kBrushless);
    rotationMotor.restoreFactoryDefaults();
    rotationMotor.setIdleMode(rotationIdle);
    rotationMotor.setSmartCurrentLimit(rotationCurrentLimit);
    rotationMotor.setInverted(rotationReversed);
    rotationMotor.enableVoltageCompensation(SwerveConstants.VoltCompensation);

    turnEncoder = rotationMotor.getEncoder();
    turnEncoder.setPositionConversionFactor(SwerveConstants.TurningEncoderPositionFactor);
    turnEncoder.setVelocityConversionFactor(SwerveConstants.TurningEncoderVelocityFactor);    
    rotationMotor.burnFlash();

    rotationPID = new PIDController(0.01, 0, 0);
    rotationPID.enableContinuousInput(0, 360);
    
    driveEncoder.setPosition(0);
  }
  public void resetEncoders(){ driveEncoder.setPosition(0); turnEncoder.setPosition(0); }
  public SwerveModuleState getState() { return new SwerveModuleState(driveEncoder.getVelocity(), getRotation2d()); }
  public SwerveModulePosition getPosition() { return new SwerveModulePosition(driveEncoder.getPosition(), getRotation2d()); }  
  public double GetModuleAngle() { return rotationEncoder.getAbsolutePosition(); }
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState, getRotation2d());
    if(Math.abs(optimizedDesiredState.speedMetersPerSecond) < 0.01){
      stopModule();
      return;
    }
    driveMotor.set(optimizedDesiredState.speedMetersPerSecond);
    rotationMotor.set(rotationPID.calculate(rotationEncoder.getAbsolutePosition(), optimizedDesiredState.angle.getDegrees()));
  }
  public void setDesiredStateOverride(SwerveModuleState desiredState) {
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState, getRotation2d());
    driveMotor.set(optimizedDesiredState.speedMetersPerSecond);
    rotationMotor.set(rotationPID.calculate(rotationEncoder.getAbsolutePosition(), optimizedDesiredState.angle.getDegrees()));
  }
  public void stopModule(){ driveMotor.stopMotor(); rotationMotor.stopMotor(); }
  public double getAngle(){ return rotationEncoder.getAbsolutePosition(); }
  public Rotation2d getRotation2d() { return Rotation2d.fromDegrees(getAngle()); }
  public double getDrivePower(){ return driveMotor.get(); }
  public double getRotationPower() { return rotationMotor.get(); }
}