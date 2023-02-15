package frc.robot.classes;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
  private final CANSparkMax driveMotor;
  private final CANSparkMax rotationMotor;
  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder rotationEncoder;
  private final SparkMaxPIDController drivePID;
  private final SparkMaxPIDController rotationPID;
  private double rotationOffset;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  public SwerveModule(String moduleName, int drivePort, IdleMode driveIdle, int driveCurrentLimit, boolean driveReversed, int rotationPort, IdleMode rotationIdle, int rotationCurrentLimit, boolean rotationReversed, double rotationOffset) {
    driveMotor = new CANSparkMax(drivePort, MotorType.kBrushless);
    driveMotor.restoreFactoryDefaults();
    driveMotor.setIdleMode(driveIdle);
    driveMotor.setSmartCurrentLimit(driveCurrentLimit);
    driveMotor.setInverted(driveReversed);
    driveEncoder = driveMotor.getEncoder();
    drivePID = driveMotor.getPIDController();
    drivePID.setFeedbackDevice(driveEncoder);
    driveEncoder.setPositionConversionFactor(SwerveConstants.DriveEncoderPositionFactor);
    driveEncoder.setVelocityConversionFactor(SwerveConstants.DriveEncoderVelocityFactor);
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
    rotationEncoder = rotationMotor.getAbsoluteEncoder(Type.kDutyCycle);
    rotationEncoder.setInverted(SwerveConstants.RotationEncoderReversed);
    rotationPID = rotationMotor.getPIDController();
    rotationPID.setFeedbackDevice(rotationEncoder);
    rotationPID.setPositionPIDWrappingEnabled(true);
    rotationEncoder.setPositionConversionFactor(SwerveConstants.RotationEncoderPositionFactor);
    rotationEncoder.setVelocityConversionFactor(SwerveConstants.RotationEncoderVelocityFactor);
    rotationPID.setPositionPIDWrappingMinInput(SwerveConstants.RotationEncoderPositionPIDMinInput);
    rotationPID.setPositionPIDWrappingMaxInput(SwerveConstants.RotationEncoderPositionPIDMaxInput);
    rotationPID.setP(SwerveConstants.RotationP);
    rotationPID.setI(SwerveConstants.RotationI);
    rotationPID.setD(SwerveConstants.RotationD);
    rotationPID.setFF(SwerveConstants.RotationFF);
    rotationPID.setOutputRange(SwerveConstants.RotationMinOutput, SwerveConstants.RotationMaxOutput);
    rotationMotor.burnFlash();
    
    this.rotationOffset = rotationOffset;
    this.desiredState.angle = new Rotation2d(rotationEncoder.getPosition());
    driveEncoder.setPosition(0);
  }
  public void resetEncoders(){ driveEncoder.setPosition(0); }
  public SwerveModuleState getState() { return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(rotationEncoder.getPosition() - rotationOffset)); }
  public SwerveModulePosition getPosition() { return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(rotationEncoder.getPosition() - rotationOffset)); }  
  public double GetModuleAngle() { return rotationEncoder.getPosition(); }
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(rotationOffset));
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState, new Rotation2d(rotationEncoder.getPosition()));
    drivePID.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    rotationPID.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
    this.desiredState = desiredState;
  }
  public void stopModule(){ setDesiredState(new SwerveModuleState(0.0, new Rotation2d(rotationEncoder.getPosition() - rotationOffset))); }
}