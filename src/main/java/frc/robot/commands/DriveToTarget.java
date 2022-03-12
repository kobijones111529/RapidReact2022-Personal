package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import si.uom.SI;

import javax.measure.Quantity;
import javax.measure.Unit;
import javax.measure.quantity.Angle;
import javax.measure.quantity.Length;
import java.util.function.Supplier;

public class DriveToTarget extends CommandBase {
  private final Drivetrain drivetrain;
  private final Supplier<Quantity<Length>> distanceFromTarget;
  private final Supplier<Quantity<Angle>> angleFromTarget;
  private final Quantity<Length> distanceTolerance;
  private final Quantity<Angle> angleTolerance;

  public DriveToTarget(Drivetrain drivetrain, Supplier<Quantity<Length>> distanceFromTarget, Supplier<Quantity<Angle>> angleFromTarget, Quantity<Length> distanceTolerance, Quantity<Angle> angleTolerance) {
    this.drivetrain = drivetrain;
    this.distanceFromTarget = distanceFromTarget;
    this.angleFromTarget = angleFromTarget;
    this.distanceTolerance = distanceTolerance;
    this.angleTolerance = angleTolerance;

    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    double moveScalar = 1;
    double turnScalar = 1;

    double distanceMeters = distanceFromTarget.get().to(SI.METRE).getValue().doubleValue();
    double angleRotations = angleFromTarget.get().to(SI.REVOLUTION).getValue().doubleValue();
    double move = moveScalar * distanceMeters;
    double turn = turnScalar * angleRotations;
    drivetrain.simpleArcadeDrive(MathUtil.clamp(move, -1, 1), MathUtil.clamp(turn, -1, 1));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.simpleArcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    Unit<Length> distanceUnit = SI.METRE;
    Unit<Angle> angleUnit = SI.REVOLUTION;
    boolean distanceReached = distanceFromTarget.get().to(distanceUnit).getValue().doubleValue() < distanceTolerance.to(distanceUnit).getValue().doubleValue();
    boolean angleReached = angleFromTarget.get().to(angleUnit).getValue().doubleValue() < angleTolerance.to(angleUnit).getValue().doubleValue();
    return distanceReached && angleReached;
  }
}
