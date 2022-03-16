package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import si.uom.SI;
import systems.uom.common.USCustomary;
import tech.units.indriya.quantity.Quantities;

import javax.measure.Quantity;
import javax.measure.quantity.Angle;
import javax.measure.quantity.Length;
import java.util.function.Supplier;

public class AutoShoot extends CommandBase {
  private final Quantity<Angle> angleOffsetTolerance = Quantities.getQuantity(5, USCustomary.DEGREE_ANGLE);

  private final Magazine magazine;
  private final Shooter shooter;
  private final Supplier<Quantity<Length>> distanceToTargetSupplier;
  private final Supplier<Quantity<Angle>> angleOffsetSupplier;

  public AutoShoot(Magazine magazine, Shooter shooter, Supplier<Quantity<Length>> distanceToTargetSupplier, Supplier<Quantity<Angle>> angleOffsetSupplier) {
    this.magazine = magazine;
    this.shooter = shooter;
    this.distanceToTargetSupplier = distanceToTargetSupplier;
    this.angleOffsetSupplier = angleOffsetSupplier;

    addRequirements(magazine, shooter);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {

  }

  @Override
  public boolean isFinished() {
    boolean angleOK = Math.abs(angleOffsetSupplier.get().to(SI.RADIAN).getValue().doubleValue()) > angleOffsetTolerance.to(SI.RADIAN).getValue().doubleValue();
    return !angleOK;
  }
}
