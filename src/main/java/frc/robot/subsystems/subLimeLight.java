package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.BooleanSupplier;
import org.photonvision.PhotonCamera;
import frc.robot.classes.LimelightHelpers;

public class subLimeLight extends SubsystemBase {
  private double horizAngleToTarget;
  // Fiducial values
  private double[] botPose = LimelightHelpers.getBotPose("");
  private double[] botPoseBlue = LimelightHelpers.getBotPose_wpiBlue("");
  private double[] botPoseRed = LimelightHelpers.getBotPose_wpiRed("");

  // Both Fiducial and RetroReflective
  private boolean hasVision = LimelightHelpers.getTV("");

  // Read the README file in thunder's limelightlib for pipeline indexes
  private int pipelineNum = 1;
  private double curPipeline = LimelightHelpers.getCurrentPipelineIndex("");

  // RetroReflective values
  private double horizontalOffset = LimelightHelpers.getTX("");
  private double verticalOffset = LimelightHelpers.getTY("");
  private double targetVertical = LimelightHelpers.getTA("");

  // Getting limelight's latency
  private double latencyPipline = LimelightHelpers.getLatency_Pipeline("");
  private double latencyCapture = LimelightHelpers.getLatency_Capture("");

  private double botPoseTotalLatency;
  private double botPoseBlueTotalLatency;
  private double botPoseRedTotalLatency;

  public subLimeLight() {
    
  }

  public Pose2d getRobotPose() {
    if (hasVision) {
        return new Pose2d(new Translation2d(getBotPoseBlue()[0], getBotPoseBlue()[1]),
                Rotation2d.fromDegrees(getBotPoseBlue()[5]));
    } else {
        return null;
    }
}

public double getHorizontalOffset() {
    horizontalOffset = LimelightHelpers.getTX("");
    return horizontalOffset;
}

public double getVerticalOffset() {
    verticalOffset = LimelightHelpers.getTX("");
    return verticalOffset;
}

public double getTargetVertical() {
    targetVertical = LimelightHelpers.getTX("");
    return targetVertical;
}

public double[] getBotPose() {
    botPose = LimelightHelpers.getBotPose("");
    if (botPose != null && botPose.length != 0) {
        return botPose;
    } else {
        return new double[] { 0, 0, 0, 0, 0, 0, 0 };
    }
}

public double[] getBotPoseRed() {
    botPoseRed = LimelightHelpers.getBotPose_wpiRed("");
    if (botPoseRed != null && botPoseRed.length != 0) {
        return botPoseRed;
    } else {
        return new double[] { 0, 0, 0, 0, 0, 0, 0 };
    }
}

public double[] getBotPoseBlue() {
    botPoseBlue = LimelightHelpers.getBotPose_wpiBlue("");
    if (botPoseBlue != null && botPoseBlue.length != 0) {
        return botPoseBlue;
    } else {
        return new double[] { 0, 0, 0, 0, 0, 0, 0 };
    }
}

public boolean getVision() {
    hasVision = LimelightHelpers.getTV("");
    return hasVision;
}

public double getLatencyPipline() {
    latencyPipline = LimelightHelpers.getLatency_Pipeline("");
    return latencyPipline;
}

public double getLatencyCapture() {
    latencyCapture = LimelightHelpers.getLatency_Capture("");
    return latencyCapture;
}

public double getLatencyBotPose() {
    var pose = LimelightHelpers.getBotPose("");
    if (pose.length != 0) {
        botPoseTotalLatency = LimelightHelpers.getBotPose("")[6];
        return botPoseTotalLatency;
    }
    return 0;
}

public double getLatencyBotPoseBlue() {
    var pose = LimelightHelpers.getBotPose_wpiBlue("");
    if (pose.length != 0) {
        botPoseBlueTotalLatency = LimelightHelpers.getBotPose_wpiBlue("")[6];
        return botPoseBlueTotalLatency;
    }
    return 0;
}

public double getLatencyBotPoseRed() {
    var pose = LimelightHelpers.getBotPose_wpiRed("");
    if (pose.length != 0) {
        botPoseRedTotalLatency = LimelightHelpers.getBotPose_wpiRed("")[6];
        return botPoseRedTotalLatency;
    }
    return 0;
}

private void updateShuffleboard() {
    if (pipelineNum == 0 || pipelineNum == 1) {
      SmartDashboard.putNumber("Vision bot pose TX", getBotPose()[0]);
      SmartDashboard.putNumber("Vision bot pose TY", getBotPose()[1]);
      SmartDashboard.putNumber("Vision bot pose RZ", getBotPose()[5]);

      SmartDashboard.putNumber("Vision bot pose Blue TX", getBotPoseBlue()[0]);
      SmartDashboard.putNumber("Vision bot pose Blue TY", getBotPoseBlue()[1]);
      SmartDashboard.putNumber("Vision bot pose Blue RZ", getBotPoseBlue()[5]);

      SmartDashboard.putNumber("Vision bot pose Red TX", getBotPoseRed()[0]);
      SmartDashboard.putNumber("Vision bot pose Red TY", getBotPoseRed()[1]);
      SmartDashboard.putNumber("Vision bot pose Red RZ", getBotPoseRed()[5]);
    } else if (pipelineNum == 2 || pipelineNum == 3) {
      SmartDashboard.putNumber("RR Tape Horizontal Offset", getHorizontalOffset());
      SmartDashboard.putNumber("RR Tape Vertical Offset", getVerticalOffset());
      SmartDashboard.putNumber("RR Tape Target Area", getTargetVertical());
    }
}

public void setPipelineNum(int pipelineNum) {
    LimelightHelpers.setPipelineIndex("", pipelineNum);
}

public double getPipelineNum() {
    curPipeline = LimelightHelpers.getCurrentPipelineIndex("");
    return this.curPipeline;
}

public boolean validTarget() {
    return Math.abs(this.horizAngleToTarget) < 29.8;
}

public double autoAlign() {
    setPipelineNum(2);
    this.horizAngleToTarget = LimelightHelpers.getTX("");
    boolean isOnTarget = isOnTarget(this.horizAngleToTarget);
    hasVision = LimelightHelpers.getTV("");
    if (hasVision && !isOnTarget && validTarget()) {
        return horizAngleToTarget;
    } else {
        return 0d;
    }
}

public boolean isOnTarget(double expectedAngle) {
    return expectedAngle < 1.0;
}
  
  @Override
  public void periodic() {
    updateShuffleboard();
  }
}