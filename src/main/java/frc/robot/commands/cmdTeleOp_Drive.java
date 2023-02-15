package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.subSwerve;

public class cmdTeleOp_Drive extends CommandBase {
  private final subSwerve swerve;
  private final DoubleSupplier XSupplier;
  private final DoubleSupplier YSupplier;
  private final DoubleSupplier rotationSupplier;
  private final Supplier<Boolean> fieldOrientedFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  
  public cmdTeleOp_Drive(subSwerve swerve, DoubleSupplier XSupplier, DoubleSupplier YSupplier, DoubleSupplier rotationSupplier, Supplier<Boolean> fieldOrientedFunction) {
    this.swerve = swerve;
    this.XSupplier = XSupplier;
    this.YSupplier = YSupplier;
    this.rotationSupplier = rotationSupplier;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.xLimiter = new SlewRateLimiter(SwerveConstants.TeleOp.MaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(SwerveConstants.TeleOp.MaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(SwerveConstants.TeleOp.MaxAngularAccelerationUnitsPerSecond);
    addRequirements(swerve);
  }
  
  @Override
  public void initialize() {}
  
  @Override
  public void execute() {
    double X = xLimiter.calculate(XSupplier.getAsDouble()) * SwerveConstants.MaxSpeedMetersPerSecond;
    double Y = yLimiter.calculate(YSupplier.getAsDouble()) * SwerveConstants.MaxSpeedMetersPerSecond;
    double R = turningLimiter.calculate(rotationSupplier.getAsDouble()) * SwerveConstants.MaxAngularSpeed;
    swerve.drive(X, Y, R, fieldOrientedFunction.get());
  }
  
  @Override
  public void end(boolean interrupted) {  }
  
  @Override
  public boolean isFinished() { return false; }
}
