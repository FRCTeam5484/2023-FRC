package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    public static final int DriverOne = 0;
    public static final int DriverTwo = 1;
  }
  public static final class SwerveConstants {
    public static final double TrackWidth = Units.inchesToMeters(17.5);
    public static final double WheelBase = Units.inchesToMeters(29);

    public static final SwerveDriveKinematics Kinematics = new SwerveDriveKinematics(
        new Translation2d(TrackWidth / 2.0, WheelBase / 2.0), // front left
        new Translation2d(TrackWidth / 2.0, -WheelBase / 2.0), // front right
        new Translation2d(-TrackWidth / 2.0, WheelBase / 2.0), // back left
        new Translation2d(-TrackWidth / 2.0, -WheelBase / 2.0) // back right
    );

    public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.1, 0.15, 0.01);

    public static final double MaxSpeed = 4.5; // meters per second
    public static final double MaxAngularVelocity = 11.5;

    public static final class TeleOp{
      public static final double MaxSpeedMetersPerSecond = 5676.0 / 60.0 * SdsModuleConfigurations.MK3_STANDARD.getDriveReduction() * SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI;
      public static final double MaxAngularSpeedRadiansPerSecond = MaxSpeedMetersPerSecond / Math.hypot(SwerveConstants.TrackWidth / 2.0, SwerveConstants.WheelBase / 2.0);
      public static final double MaxAccelerationUnitsPerSecond = 3;
      public static final double MaxAngularAccelerationUnitsPerSecond = 3;
    }
       
    public static final class FrontLeft{
      public static final int DriveMotorPort = 4;
      public static final int TurningMotorPort = 3;
      public static final IdleMode DriveIdleMode = IdleMode.kBrake;
      public static final IdleMode TurnIdleMode = IdleMode.kBrake;
      public static final int DriveCurrentLimit = 60;
      public static final int TurnCurrentLimit = 60;
      public static final boolean DriveMotorReversed = false;
      public static final boolean TurningMotorReversed = false;
      public static final int TurnAbsoluteEncoderPort = 1;
      public static final boolean TurnAbsoluteEncoderReversed = true;
      public static final double TurnAbsoluteEncoderOffsetRad = -Math.toRadians(0);
    }

    public static final class FrontRight{
      public static final int DriveMotorPort = 2;
      public static final int TurningMotorPort = 1;
      public static final IdleMode DriveIdleMode = IdleMode.kBrake;
      public static final IdleMode TurnIdleMode = IdleMode.kBrake;
      public static final int DriveCurrentLimit = 60;
      public static final int TurnCurrentLimit = 60;
      public static final boolean DriveMotorReversed = false;
      public static final boolean TurningMotorReversed = false;
      public static final int TurnAbsoluteEncoderPort = 5;
      public static final boolean TurnAbsoluteEncoderReversed = true;
      public static final double TurnAbsoluteEncoderOffsetRad = -Math.toRadians(0);
    }

    public static final class BackLeft{
      public static final int DriveMotorPort = 8;
      public static final int TurningMotorPort = 7;
      public static final IdleMode DriveIdleMode = IdleMode.kBrake;
      public static final IdleMode TurnIdleMode = IdleMode.kBrake;
      public static final int DriveCurrentLimit = 60;
      public static final int TurnCurrentLimit = 60;
      public static final boolean DriveMotorReversed = false;
      public static final boolean TurningMotorReversed = false;
      public static final int TurnAbsoluteEncoderPort = 3;
      public static final boolean TurnAbsoluteEncoderReversed = true;
      public static final double TurnAbsoluteEncoderOffsetRad = -Math.toRadians(0);
    }

    public static final class BackRight{
      public static final int DriveMotorPort = 6;
      public static final int TurningMotorPort = 5;
      public static final IdleMode DriveIdleMode = IdleMode.kBrake;
      public static final IdleMode TurnIdleMode = IdleMode.kBrake;
      public static final int DriveCurrentLimit = 60;
      public static final int TurnCurrentLimit = 60;
      public static final boolean DriveMotorReversed = false;
      public static final boolean TurningMotorReversed = false;
      public static final int TurnAbsoluteEncoderPort = 7;
      public static final boolean TurnAbsoluteEncoderReversed = true;
      public static final double TurnAbsoluteEncoderOffsetRad = -Math.toRadians(0);
    }
  }
  public static final class ArmAngleConstants {
    public static final int Port = 9;
    public static final boolean Reversed = false;
    public static final IdleMode Mode = IdleMode.kBrake;
    public static final double PowerFactor = 0.8;
    public static final int PowerLimit = 60;
    
    public static final double limitPositionHigh = 13;
    public static final double limitPositionLow = 81;
    public static final double HumanFeedPosition = 0;
    public static final double GroundPosition = 0;
    public static final double MidPosition = 0;
    public static final double HighPosition = 0;
  }
  public static final class ArmExtensionConstants {
    public static final int Port = 10;
    public static final boolean Reversed = false;
    public static final IdleMode Mode = IdleMode.kBrake;
    public static final double PowerFactor = 0.5;
    public static final int PowerLimit = 60;

    public static final double limitOpen = 165;
    public static final double limitClosed = 2;
    public static final double HumanFeedPosition = 0;
    public static final double GroundPosition = 0;
    public static final double MidPosition = 0;
    public static final double HighPosition = 0;
  }
  public static final class ClawConstants {
    public static final int Port = 11;
    public static final boolean Reversed = false;
    public static final IdleMode Mode = IdleMode.kBrake;
    public static final double PowerFactor = 0.2;
    public static final int PowerLimit = 40;

    public static final double openLimit = 0;
    public static final double closeLimit = 10;
    public static final int Cube = 8;
    public static final int Cone = 2;
  }
  public static final class ServoConstants{
    public static final int servoPort = 0;
    
    public static final int coneUp = 0;
    public static final int coneDown = 90;
    public static final int cubeUp = -90;
    public static final int cubeDown = 179;
  }
}
