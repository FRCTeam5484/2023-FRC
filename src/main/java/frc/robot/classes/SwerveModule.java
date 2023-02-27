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
import frc.robot.Constants.SwerveConstants.FrontLeft;

public class SwerveModule {
  private final String moduleName;
  private final CANSparkMax driveMotor;
  private final CANSparkMax rotationMotor;
  private final RelativeEncoder driveEncoder;
  private final CANCoder rotationEncoder;
  private final SparkMaxPIDController drivePID;
  //private final SparkMaxPIDController rotationPID;
  private final PIDController rotationPID;

  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

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

    drivePID = driveMotor.getPIDController();
    drivePID.setFeedbackDevice(driveEncoder);
    drivePID.setP(SwerveConstants.DriveP);
    drivePID.setI(SwerveConstants.DriveI);
    drivePID.setD(SwerveConstants.DriveD);
    drivePID.setFF(SwerveConstants.DriveFF);
    drivePID.setOutputRange(SwerveConstants.DriveMinOutput, SwerveConstants.DriveMaxOutput);
    driveMotor.burnFlash();

    rotationMotor = new CANSparkMax(rotationPort, MotorType.kBrushless);
    rotationMotor.restoreFactoryDefaults();
    rotationMotor.setIdleMode(rotationIdle);
    rotationMotor.setSmartCurrentLimit(rotationCurrentLimit);
    rotationMotor.setInverted(rotationReversed);
    rotationMotor.enableVoltageCompensation(SwerveConstants.VoltCompensation);
    rotationMotor.burnFlash();

    rotationPID = new PIDController(0.01, 0, 0);
    rotationPID.enableContinuousInput(0, 360);
    
    this.desiredState.angle = new Rotation2d(rotationEncoder.getAbsolutePosition());//getPosition());
    driveEncoder.setPosition(0);
  }
  public void resetEncoders(){ driveEncoder.setPosition(0); }
  public SwerveModuleState getState() { return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromDegrees(rotationEncoder.getAbsolutePosition())); }
  public SwerveModulePosition getPosition() { return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromDegrees(rotationEncoder.getAbsolutePosition())); }  
  public double GetModuleAngle() { return rotationEncoder.getAbsolutePosition(); }
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(rotationEncoder.getAbsolutePosition()));
    if(optimizedDesiredState.speedMetersPerSecond < 0.05){
      stopModule();
      return;
    }
    driveMotor.set(optimizedDesiredState.speedMetersPerSecond);
    rotationMotor.set(rotationPID.calculate(rotationEncoder.getAbsolutePosition(), optimizedDesiredState.angle.getDegrees()));
    this.desiredState = desiredState;
  }
  public void setDesiredStateOverride(SwerveModuleState desiredState) {
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(rotationEncoder.getAbsolutePosition()));
    driveMotor.set(optimizedDesiredState.speedMetersPerSecond);
    rotationMotor.set(rotationPID.calculate(rotationEncoder.getAbsolutePosition(), optimizedDesiredState.angle.getDegrees()));
    this.desiredState = desiredState;
  }
  public void stopModule(){ driveMotor.stopMotor(); rotationMotor.stopMotor(); } //setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(rotationEncoder.getAbsolutePosition()))); }
  public double getAngleRaw(){ return rotationEncoder.getAbsolutePosition(); }
  public double getDrivePower(){ return driveMotor.get(); }
  public double getRotationPower() { return rotationMotor.get(); }
}