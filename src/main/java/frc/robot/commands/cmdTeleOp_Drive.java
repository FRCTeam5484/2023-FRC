package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.subDriveTrain;

public class cmdTeleOp_Drive extends CommandBase {
  private final subDriveTrain driveTrain;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;
  private final Supplier<Boolean> fieldOrientedFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  
  public cmdTeleOp_Drive(subDriveTrain driveTrain, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier, Supplier<Boolean> fieldOrientedFunction) {
    this.driveTrain = driveTrain;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.xLimiter = new SlewRateLimiter(SwerveConstants.TeleOp.MaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(SwerveConstants.TeleOp.MaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(SwerveConstants.TeleOp.MaxAngularAccelerationUnitsPerSecond);
    addRequirements(driveTrain);
  }
  
  @Override
  public void initialize() {}
  
  @Override
  public void execute() {
    double X = Math.abs(translationXSupplier.getAsDouble()) > 0.1 ? translationXSupplier.getAsDouble() : 0.0;
    double Y = Math.abs(translationYSupplier.getAsDouble()) > 0.1 ? translationYSupplier.getAsDouble() : 0.0;
    double R = Math.abs(rotationSupplier.getAsDouble()) > 0.1 ? rotationSupplier.getAsDouble() : 0.0;
    X = xLimiter.calculate(X) * SwerveConstants.TeleOp.MaxSpeedMetersPerSecond;
    Y = yLimiter.calculate(Y) * SwerveConstants.TeleOp.MaxSpeedMetersPerSecond;
    R = turningLimiter.calculate(R) * SwerveConstants.TeleOp.MaxAngularSpeedRadiansPerSecond;
    driveTrain.drive(X, Y, R, fieldOrientedFunction.get());
  }
  
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopModules();
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
