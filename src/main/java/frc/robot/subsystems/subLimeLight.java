package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.BooleanSupplier;
import org.photonvision.PhotonCamera;

public class subLimeLight extends SubsystemBase {
  private PhotonCamera camera;
  public subLimeLight() {
    camera = new PhotonCamera("hhCam");
  }
  public PhotonCamera getCamera() { return camera; }
  public BooleanSupplier hasTargetBooleanSupplier() { return () -> camera.getLatestResult().hasTargets(); }
  public final double CameraHeightMeters = Units.inchesToMeters(11);
  public final double TargetHeightMeters = Units.inchesToMeters(25);
  public final double CameraPitchRadians = Units.degreesToRadians(0);
  public void takeSnapshot() { camera.takeInputSnapshot(); }

  @Override
  public void periodic() {
    var result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    SmartDashboard.putBoolean("Has target", hasTargets);
    if (hasTargets) {
      SmartDashboard.putNumber("tag ID", result.getBestTarget().getFiducialId());
      SmartDashboard.putNumber("pose ambiguity", result.getBestTarget().getPoseAmbiguity());
      Transform3d bestCameraToTarget = result.getBestTarget().getBestCameraToTarget();
      SmartDashboard.putNumber("x (roll)", Units.radiansToDegrees(bestCameraToTarget.getRotation().getX()));
      SmartDashboard.putNumber("y (pitch)", Units.radiansToDegrees(bestCameraToTarget.getRotation().getY()));
      SmartDashboard.putNumber("z (yaw)", Units.radiansToDegrees(bestCameraToTarget.getRotation().getZ()));
      SmartDashboard.putNumber("x", bestCameraToTarget.getX());
      SmartDashboard.putNumber("y", bestCameraToTarget.getY());
      SmartDashboard.putNumber("z", bestCameraToTarget.getZ());
    }
  }
}