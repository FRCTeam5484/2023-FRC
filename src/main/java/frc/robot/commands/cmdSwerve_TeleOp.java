package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.subSwerve;

public class cmdSwerve_TeleOp extends CommandBase {
  private final subSwerve swerve;
  private final DoubleSupplier XSupplier;
  private final DoubleSupplier YSupplier;
  private final DoubleSupplier rotationSupplier;
  private final BooleanSupplier boost;
  private final SlewRateLimiter xLimiter, yLimiter, rotationLimiter;
  
  public cmdSwerve_TeleOp(subSwerve swerve, DoubleSupplier XSupplier, DoubleSupplier YSupplier, DoubleSupplier rotationSupplier, BooleanSupplier boost) {
    this.swerve = swerve;
    this.XSupplier = XSupplier;
    this.YSupplier = YSupplier;
    this.rotationSupplier = rotationSupplier;
    this.boost = boost;
    this.xLimiter = new SlewRateLimiter(3.5);
    this.yLimiter = new SlewRateLimiter(3.5);
    this.rotationLimiter = new SlewRateLimiter(4.5);
    addRequirements(swerve);
  }
  
  @Override
  public void initialize() {}
  
  @Override
  public void execute() {
    double xSpeed = XSupplier.getAsDouble();
    double ySpeed = YSupplier.getAsDouble();
    double rotationSpeed = rotationSupplier.getAsDouble();

    if(boost.getAsBoolean()){
      xSpeed = xLimiter.calculate(xSpeed)*(SwerveConstants.TeleOp.DriveSpeedFactor+0.3);
      ySpeed = yLimiter.calculate(ySpeed)*(SwerveConstants.TeleOp.DriveSpeedFactor+0.3);
      rotationSpeed = rotationLimiter.calculate(rotationSpeed)*SwerveConstants.TeleOp.RotationSpeedFactor;
    }
    else {
      xSpeed = xLimiter.calculate(xSpeed)*SwerveConstants.TeleOp.DriveSpeedFactor;
      ySpeed = yLimiter.calculate(ySpeed)*SwerveConstants.TeleOp.DriveSpeedFactor;
      rotationSpeed = rotationLimiter.calculate(rotationSpeed)*SwerveConstants.TeleOp.RotationSpeedFactor;
    }

    
    
    swerve.drive(xSpeed, ySpeed, rotationSpeed);
  }
  
  @Override
  public void end(boolean interrupted) {  }
  
  @Override
  public boolean isFinished() { return false; }
}