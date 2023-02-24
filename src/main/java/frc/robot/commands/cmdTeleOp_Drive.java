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
  private final SlewRateLimiter xLimiter, yLimiter, rotationLimiter;
  
  public cmdTeleOp_Drive(subSwerve swerve, DoubleSupplier XSupplier, DoubleSupplier YSupplier, DoubleSupplier rotationSupplier, Supplier<Boolean> fieldOrientedFunction) {
    this.swerve = swerve;
    this.XSupplier = XSupplier;
    this.YSupplier = YSupplier;
    this.rotationSupplier = rotationSupplier;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.xLimiter = new SlewRateLimiter(2);
    this.yLimiter = new SlewRateLimiter(2);
    this.rotationLimiter = new SlewRateLimiter(3);
    addRequirements(swerve);
  }
  
  @Override
  public void initialize() {}
  
  @Override
  public void execute() {
    double xSpeed = XSupplier.getAsDouble();
    double ySpeed = YSupplier.getAsDouble();
    double rotationSpeed = rotationSupplier.getAsDouble();

    xSpeed = xLimiter.calculate(xSpeed)*SwerveConstants.TeleOp.DriveSpeedFactor;
    ySpeed = yLimiter.calculate(ySpeed)*SwerveConstants.TeleOp.DriveSpeedFactor;
    rotationSpeed = rotationLimiter.calculate(rotationSpeed)*SwerveConstants.TeleOp.RotationSpeedFactor;
    
    swerve.drive(xSpeed, ySpeed, rotationSpeed, fieldOrientedFunction.get());
  }
  
  @Override
  public void end(boolean interrupted) {  }
  
  @Override
  public boolean isFinished() { return false; }
}