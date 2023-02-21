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
  
  public cmdTeleOp_Drive(subSwerve swerve, DoubleSupplier XSupplier, DoubleSupplier YSupplier, DoubleSupplier rotationSupplier, Supplier<Boolean> fieldOrientedFunction) {
    this.swerve = swerve;
    this.XSupplier = XSupplier;
    this.YSupplier = YSupplier;
    this.rotationSupplier = rotationSupplier;
    this.fieldOrientedFunction = fieldOrientedFunction;
    addRequirements(swerve);
  }
  
  @Override
  public void initialize() {}
  
  @Override
  public void execute() {
    swerve.drive(XSupplier.getAsDouble()*SwerveConstants.TeleOp.DriveSpeedFactor, YSupplier.getAsDouble()*SwerveConstants.TeleOp.DriveSpeedFactor, rotationSupplier.getAsDouble()*SwerveConstants.TeleOp.RotationSpeedFactor, fieldOrientedFunction.get());
  }
  
  @Override
  public void end(boolean interrupted) {  }
  
  @Override
  public boolean isFinished() { return false; }
}
