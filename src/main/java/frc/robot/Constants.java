package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    public static final int DriverOne = 0;
    public static final int DriverTwo = 1;
  }
  public static final class DriveConstants {
    public static final double TrackWidth = Units.inchesToMeters(23.5);
    public static final double WheelBase = Units.inchesToMeters(23.5);
    //public static final SwerveDriveKinematics DriveKinematics = new SwerveDriveKinematics(
    //        new Translation2d(-DriveConstants.Base.WheelBase / 2, DriveConstants.Base.TrackWidth / 2), //front left
    //        new Translation2d(-DriveConstants.Base.WheelBase / 2, -DriveConstants.Base.TrackWidth / 2), //front right
    //        new Translation2d(DriveConstants.Base.WheelBase / 2, DriveConstants.Base.TrackWidth / 2), //back left
    //        new Translation2d(DriveConstants.Base.WheelBase / 2, -DriveConstants.Base.TrackWidth / 2)); //backright

    public static final class FrontLeft{
      public static final int DriveMotorPort = 4;
      public static final int TurningMotorPort = 3;
      public static final IdleMode DriveIdleMode = IdleMode.kBrake;
      public static final IdleMode TurnIdleMode = IdleMode.kBrake;
      public static final int DriveCurrentLimit = 35;
      public static final int TurnCurrentLimit = 35;
      public static final boolean DriveMotorReversed = false;
      public static final boolean TurningMotorReversed = true;
      public static final int TurnAbsoluteEncoderPort = 3;
      public static final boolean TurnAbsoluteEncoderReversed = true;
      public static final double TurnAbsoluteEncoderOffsetRad = -106.5;
    }

    public static final class FrontRight{
      public static final int DriveMotorPort = 2;
      public static final int TurningMotorPort = 1;
      public static final IdleMode DriveIdleMode = IdleMode.kBrake;
      public static final IdleMode TurnIdleMode = IdleMode.kBrake;
      public static final int DriveCurrentLimit = 35;
      public static final int TurnCurrentLimit = 35;
      public static final boolean DriveMotorReversed = false;
      public static final boolean TurningMotorReversed = true;
      public static final int TurnAbsoluteEncoderPort = 1;
      public static final boolean TurnAbsoluteEncoderReversed = true;
      public static final double TurnAbsoluteEncoderOffsetRad = -128;
    }

    public static final class BackLeft{
      public static final int DriveMotorPort = 8;
      public static final int TurningMotorPort = 7;
      public static final IdleMode DriveIdleMode = IdleMode.kBrake;
      public static final IdleMode TurnIdleMode = IdleMode.kBrake;
      public static final int DriveCurrentLimit = 35;
      public static final int TurnCurrentLimit = 35;
      public static final boolean DriveMotorReversed = true;
      public static final boolean TurningMotorReversed = true;
      public static final int TurnAbsoluteEncoderPort = 7;
      public static final boolean TurnAbsoluteEncoderReversed = true;
      public static final double TurnAbsoluteEncoderOffsetRad = -3;
    }

    public static final class BackRight{
      public static final int DriveMotorPort = 6;
      public static final int TurningMotorPort = 5;
      public static final IdleMode DriveIdleMode = IdleMode.kBrake;
      public static final IdleMode TurnIdleMode = IdleMode.kBrake;
      public static final int DriveCurrentLimit = 35;
      public static final int TurnCurrentLimit = 35;
      public static final boolean DriveMotorReversed = true;
      public static final boolean TurningMotorReversed = true;
      public static final int TurnAbsoluteEncoderPort = 5;
      public static final boolean TurnAbsoluteEncoderReversed = true;
      public static final double TurnAbsoluteEncoderOffsetRad = -64;
    }
  }
  public static final class ArmAngleConstants {
    public static final int Port = 9;
    public static final boolean Reversed = false;
    public static final IdleMode Mode = IdleMode.kBrake;
    public static final double PowerFactor = 0.5;
    public static final int PowerLimit = 35;
    
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

    public static final int OpenLimit = 0;
    public static final int CloseLimit = 10;
    public static final int Cube = 8;
    public static final int Cone = 2;
  }
}
