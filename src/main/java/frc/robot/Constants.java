package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    public static final int DriverOne = 0;
    public static final int DriverTwo = 1;
  }
  public static final class SwerveConstants {
    public static final class TeleOp{
      public static final double DriveSpeedFactor = 0.4;
      public static final double RotationSpeedFactor = 0.8;
    }
    public static final class Auto{
      public static final double MaxSpeedMetersPerSecond = 3;
      public static final double MaxAccelerationMetersPerSecondSquared = 3;
      public static final double MaxAngularSpeedRadiansPerSecond = Math.PI;
      public static final double MaxAngularSpeedRadiansPerSecondSquared = Math.PI;

      public static final double PXController = 1;
      public static final double PYController = 1;
      public static final double PThetaController = 1;

      public static final TrapezoidProfile.Constraints ThetaControllerConstraints = new TrapezoidProfile.Constraints(MaxAngularSpeedRadiansPerSecond, MaxAngularSpeedRadiansPerSecondSquared);
    }
    public static final boolean GyroReversed = false;
    public static final double TrackWidth = Units.inchesToMeters(20);
    public static final double WheelBase = Units.inchesToMeters(32);
    public static final double WheelDiameterMeters = 0.10033;
    public static final double VoltCompensation = 12.6;
    public static final SwerveDriveKinematics SwerveKinematics = new SwerveDriveKinematics(
        new Translation2d(-TrackWidth / 2.0, WheelBase / 2.0), // front left
        new Translation2d(-TrackWidth / 2.0, -WheelBase / 2.0), // front right
        new Translation2d(TrackWidth / 2.0, WheelBase / 2.0), // back left
        new Translation2d(TrackWidth / 2.0, -WheelBase / 2.0) // back right
    );
    public static final double MaxSpeedMetersPerSecond = 4.8;
    public static final double MaxAngularSpeed = 2 * Math.PI;
    public static final double DriveMotorFreeSpeedRps = 5676 / 60;
    public static final double WheelCircumferenceMeters = WheelDiameterMeters * Math.PI;
    public static final double DriveMotorReduction = 1/6.86;
    public static final double DriveWheelFreeSpeedRps = (DriveMotorFreeSpeedRps * WheelCircumferenceMeters) / DriveMotorReduction;
    public static final double DriveEncoderPositionFactor = (WheelDiameterMeters * Math.PI) / DriveMotorReduction;
    public static final double DriveEncoderVelocityFactor = ((WheelDiameterMeters * Math.PI) / DriveMotorReduction) / 60.0;
    public static final double RotationEncoderPositionFactor = (2 * Math.PI);
    public static final double RotationEncoderVelocityFactor = (2 * Math.PI) / 60.0;
    public static final double RotationEncoderPositionPIDMinInput = 0;
    public static final double RotationEncoderPositionPIDMaxInput = RotationEncoderPositionFactor;
    public static final boolean RotationEncoderReversed = true;

    public static final double DriveP = 0.04;
    public static final double DriveI = 0;
    public static final double DriveD = 0;
    public static final double DriveFF = 1 / DriveWheelFreeSpeedRps;
    public static final double DriveMinOutput = -1;
    public static final double DriveMaxOutput = 1;

    public static final double RotationP = 1;
    public static final double RotationI = 0;
    public static final double RotationD = 0;
    public static final double RotationFF = 0;
    public static final double RotationMinOutput = -1;
    public static final double RotationMaxOutput = 1;
       
    public static final class FrontLeft{
      public static final int DriveMotorPort = 4;
      public static final IdleMode DriveIdleMode = IdleMode.kBrake;
      public static final int DriveCurrentLimit = 50;
      public static final boolean DriveMotorReversed = false;
      public static final int RotationMotorPort = 3;
      public static final IdleMode RotationIdleMode = IdleMode.kBrake;
      public static final int RotationCurrentLimit = 20;
      public static final boolean RotationMotorReversed = true;
      public static final int RotationEncoderPort = 7;
      public static final double RotationAbsoluteEncoderOffset = -42;
      public static final boolean RotationEncoderReversed = true;
    }

    public static final class FrontRight{
      public static final int DriveMotorPort = 2;
      public static final IdleMode DriveIdleMode = IdleMode.kBrake;
      public static final int DriveCurrentLimit = 50;
      public static final boolean DriveMotorReversed = false;
      public static final int RotationMotorPort = 1;
      public static final IdleMode RotationIdleMode = IdleMode.kBrake;
      public static final int RotationCurrentLimit = 20;
      public static final boolean RotationMotorReversed = true;
      public static final int RotationEncoderPort = 5;
      public static final double RotationAbsoluteEncoderOffset = -154;
      public static final boolean RotationEncoderReversed = true;
    }

    public static final class BackLeft{
      public static final int DriveMotorPort = 8;
      public static final IdleMode DriveIdleMode = IdleMode.kBrake;
      public static final int DriveCurrentLimit = 50;
      public static final boolean DriveMotorReversed = false;
      public static final int RotationMotorPort = 7;
      public static final IdleMode RotationIdleMode = IdleMode.kBrake;
      public static final int RotationCurrentLimit = 20;
      public static final boolean RotationMotorReversed = true;
      public static final int RotationEncoderPort = 3;
      public static final double RotationAbsoluteEncoderOffset = -17;
      public static final boolean RotationEncoderReversed = true;
    }

    public static final class BackRight{
      public static final int DriveMotorPort = 6;
      public static final IdleMode DriveIdleMode = IdleMode.kBrake;
      public static final int DriveCurrentLimit = 50;
      public static final boolean DriveMotorReversed = false;
      public static final int RotationMotorPort = 5;      
      public static final IdleMode RotationIdleMode = IdleMode.kBrake;      
      public static final int RotationCurrentLimit = 20;      
      public static final boolean RotationMotorReversed = true;
      public static final int RotationEncoderPort = 1;
      public static final double RotationAbsoluteEncoderOffset = -91;
      public static final boolean RotationEncoderReversed = true;
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
    public static final double PowerFactor = 1;
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
    
    public static final double coneUp = 0;//
    public static final double coneDown = 60;//
    public static final double cubeUp = 135;//
    public static final double cubeDown = 180;//
  }
  public static final class PneumaticConstants{
    public final static int PneumaticHubId = 1;
      public final static int IntakeSolenoid = 0;
      // Pressure
      public final static double MinimumPressure = 100.0;
      public final static double MaximumPressure = 120.0;
  }
}