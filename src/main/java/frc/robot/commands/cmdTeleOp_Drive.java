package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.subDriveTrain;

public class cmdTeleOp_Drive extends CommandBase {
  private final subDriveTrain driveTrain;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;
  
  public cmdTeleOp_Drive(subDriveTrain driveTrain, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
    this.driveTrain = driveTrain;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;

    addRequirements(driveTrain);
  }

  
  @Override
  public void initialize() {}

  
  @Override
  public void execute() {
    driveTrain.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
              translationXSupplier.getAsDouble(),
              translationYSupplier.getAsDouble(),
              rotationSupplier.getAsDouble(),
              driveTrain.getGyroscopeRotation()
      ));
  }

  
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
