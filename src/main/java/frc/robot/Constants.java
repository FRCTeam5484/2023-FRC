package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    public static final int DriverOne = 0;
    public static final int DriverTwo = 1;
  }
  public static final class DriveConstants {
    public static final double TrackWidth = Units.inchesToMeters(17.5);
    public static final double WheelBase = Units.inchesToMeters(29);

    public static final double MaxVoltage = 12.0;
    public static final double MaxVelocityMetersPerSecond = 5676.0 / 60.0 * SdsModuleConfigurations.MK3_STANDARD.getDriveReduction() * SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI;
    public static final double MaxAngularVelocityRadiansPerSecond = MaxVelocityMetersPerSecond / Math.hypot(DriveConstants.TrackWidth / 2.0, DriveConstants.WheelBase / 2.0);
    
    public static final class FrontLeft{
      public static final int DriveMotorPort = 4;
      public static final int TurningMotorPort = 3;
      public static final IdleMode DriveIdleMode = IdleMode.kBrake;
      public static final IdleMode TurnIdleMode = IdleMode.kBrake;
      public static final int DriveCurrentLimit = 35;
      public static final int TurnCurrentLimit = 35;
      public static final boolean DriveMotorReversed = false;
      public static final boolean TurningMotorReversed = false;
      public static final int TurnAbsoluteEncoderPort = 1;
      public static final boolean TurnAbsoluteEncoderReversed = true;
      public static final double TurnAbsoluteEncoderOffsetRad = -Math.toRadians(218);
    }

    public static final class FrontRight{
      public static final int DriveMotorPort = 2;
      public static final int TurningMotorPort = 1;
      public static final IdleMode DriveIdleMode = IdleMode.kBrake;
      public static final IdleMode TurnIdleMode = IdleMode.kBrake;
      public static final int DriveCurrentLimit = 35;
      public static final int TurnCurrentLimit = 35;
      public static final boolean DriveMotorReversed = false;
      public static final boolean TurningMotorReversed = false;
      public static final int TurnAbsoluteEncoderPort = 5;
      public static final boolean TurnAbsoluteEncoderReversed = true;
      public static final double TurnAbsoluteEncoderOffsetRad = -Math.toRadians(336);
    }

    public static final class BackLeft{
      public static final int DriveMotorPort = 8;
      public static final int TurningMotorPort = 7;
      public static final IdleMode DriveIdleMode = IdleMode.kBrake;
      public static final IdleMode TurnIdleMode = IdleMode.kBrake;
      public static final int DriveCurrentLimit = 35;
      public static final int TurnCurrentLimit = 35;
      public static final boolean DriveMotorReversed = false;
      public static final boolean TurningMotorReversed = false;
      public static final int TurnAbsoluteEncoderPort = 3;
      public static final boolean TurnAbsoluteEncoderReversed = true;
      public static final double TurnAbsoluteEncoderOffsetRad = -Math.toRadians(199);
    }

    public static final class BackRight{
      public static final int DriveMotorPort = 6;
      public static final int TurningMotorPort = 5;
      public static final IdleMode DriveIdleMode = IdleMode.kBrake;
      public static final IdleMode TurnIdleMode = IdleMode.kBrake;
      public static final int DriveCurrentLimit = 35;
      public static final int TurnCurrentLimit = 35;
      public static final boolean DriveMotorReversed = false;
      public static final boolean TurningMotorReversed = false;
      public static final int TurnAbsoluteEncoderPort = 7;
      public static final boolean TurnAbsoluteEncoderReversed = true;
      public static final double TurnAbsoluteEncoderOffsetRad = -Math.toRadians(272);
    }
  }
  public static final class ArmAngleConstants {
    public static final int Port = 9;
    public static final boolean Reversed = false;
    public static final IdleMode Mode = IdleMode.kBrake;
    public static final double PowerFactor = 0.25;
    public static final int PowerLimit = 35;
    
    public static final int limitPositionHigh = 80;
    public static final int limitPositionLow = 20;
    public static final int GroundPosition = 0;
    public static final int HumanFeedPosition = 0;
    public static final int MidPosition = 0;
    public static final int HighPosition = 0;
  }
  public static final class ArmExtensionConstants {
    public static final int Port = 10;
    public static final boolean Reversed = false;
    public static final IdleMode Mode = IdleMode.kBrake;
    public static final double PowerFactor = 0.5;
    public static final int PowerLimit = 35;

    public static final int limitOpen = 0;
    public static final int limitClosed = 0;
    public static final int GroundPosition = 0;
    public static final int HumanFeedPosition = 0;
    public static final int MidPosition = 0;
    public static final int HighPosition = 0;
  }
  public static final class ClawConstants {
    public static final int Port = 11;
    public static final boolean Reversed = false;
    public static final IdleMode Mode = IdleMode.kBrake;
    public static final double PowerFactor = 0.5;
    public static final int PowerLimit = 35;

    public static final int limitOpen = 0;
    public static final int limitClosed = 0;
    public static final int OpenLimit = 0;
    public static final int CloseLimit = 10;
    public static final int Cube = 8;
    public static final int Cone = 2;
  }
}
