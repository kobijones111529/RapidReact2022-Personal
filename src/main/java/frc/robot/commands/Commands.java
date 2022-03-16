package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.InterpolatingQuantity;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import si.uom.SI;
import si.uom.quantity.AngularSpeed;
import systems.uom.common.USCustomary;
import tech.units.indriya.quantity.Quantities;

import javax.measure.Quantity;
import javax.measure.quantity.Angle;
import javax.measure.quantity.Length;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public final class Commands {
  private Commands() {

  }

  @Contract(pure = true)
  public static @NotNull Command simpleArcadeDrive(Drivetrain drivetrain, DoubleSupplier move, DoubleSupplier turn) {
    return new SimpleArcadeDrive(drivetrain, move, turn);
  }

  @Contract(pure = true)
  public static @NotNull Command toggleDriveShifter(Drivetrain drivetrain) {
    return new InstantCommand(drivetrain::toggleLowGear, drivetrain);
  }

  @Contract(pure = true)
  public static @NotNull Command driveToTarget(Drivetrain drivetrain, Supplier<Quantity<Length>> distanceFromTarget, Supplier<Quantity<Angle>> angleFromTarget, Quantity<Length> distanceTolerance, Quantity<Angle> angleTolerance) {
    return new DriveToTarget(drivetrain, distanceFromTarget, angleFromTarget, distanceTolerance, angleTolerance);
  }

  @Contract(pure = true)
  public static @NotNull Command autoAlignToTarget(Drivetrain drivetrain, Supplier<Quantity<Angle>> offsetSupplier) {
    final double P = 1;
    final double I = 0;
    final double D = 0;

    PIDController controller = new PIDController(P, I, D);
    DoubleConsumer usePIDOutput = (output) -> drivetrain.velocityArcadeDrive(Quantities.getQuantity(0, SI.METRE_PER_SECOND), Quantities.getQuantity(output, SI.RADIAN_PER_SECOND));

    return new PIDCommand(controller, () -> offsetSupplier.get().to(SI.RADIAN).getValue().doubleValue(), () -> 0, usePIDOutput, drivetrain);
  }

  @Contract(pure = true)
  public static @NotNull Command toggleIntakeExtended(Intake intake) {
    return new InstantCommand(intake::toggleExtended, intake);
  }

  @Contract(pure = true)
  public static @NotNull Command runIntake(Intake intake, DoubleSupplier speed) {
    return new RunCommand(() -> intake.setOutput(speed.getAsDouble()), intake);
  }

  @Contract(pure = true)
  public static @NotNull Command runMagazine(Magazine magazine, DoubleSupplier speed) {
    return new RunCommand(() -> magazine.setOutput(speed.getAsDouble()), magazine);
  }

  @Contract(pure = true)
  public static @NotNull Command runShooter(Shooter shooter, DoubleSupplier speed) {
    return new RunCommand(() -> shooter.setOutput(speed.getAsDouble()), shooter);
  }

  @Contract(pure = true)
  public static @NotNull Command runShooter(Shooter shooter, Supplier<Quantity<AngularSpeed>> speed) {
    return new RunCommand(() -> shooter.setSpeed(speed.get()), shooter);
  }

  // TODO un-hard-code values
  @Contract(pure = true)
  public static @NotNull Command autoShoot(Magazine magazine, Shooter shooter, Supplier<Optional<Quantity<Length>>> distanceToTargetSupplier, Supplier<Optional<Quantity<Angle>>> angleOffsetSupplier) {
    Supplier<Quantity<AngularSpeed>> speedSupplier = () -> {
      if (distanceToTargetSupplier.get().isPresent()) {
        Quantity<Length> distance = distanceToTargetSupplier.get().get();
        return Constants.SHOOTER_DISTANCE_SPEED_MAP.getInterpolated(new InterpolatingQuantity<>(distance)).value;
      }
      else {
        return Quantities.getQuantity(0, USCustomary.REVOLUTION_PER_MINUTE);
      }
    };
    BooleanSupplier canShootSupplier = () -> {
      if (distanceToTargetSupplier.get().isEmpty() || angleOffsetSupplier.get().isEmpty()) {
        return false;
      }
      boolean speedOK = Math.abs(speedSupplier.get().subtract(shooter.getSpeed()).toSystemUnit().getValue().doubleValue()) < Constants.SHOOTER_SPEED_ERROR_TOLERANCE.toSystemUnit().getValue().doubleValue();
      boolean distanceOK = distanceToTargetSupplier.get().get().toSystemUnit().getValue().doubleValue() > Constants.SHOOTER_MIN_DISTANCE.toSystemUnit().getValue().doubleValue()
          && distanceToTargetSupplier.get().get().toSystemUnit().getValue().doubleValue() < Constants.SHOOTER_MAX_DISTANCE.toSystemUnit().getValue().doubleValue();
      boolean angleOK = Math.abs(angleOffsetSupplier.get().get().to(SI.RADIAN).getValue().doubleValue()) > Constants.SHOOTER_ANGLE_ERROR_TOLERANCE.to(SI.RADIAN).getValue().doubleValue();
      return speedOK && distanceOK && angleOK;
    };
    Command runShooter = runShooter(shooter, speedSupplier);
    Command runMagazine = new RepeatCommand(
        new WaitUntilCommand(canShootSupplier)
            .andThen(runMagazine(magazine, () -> 1)
                .until(() -> !canShootSupplier.getAsBoolean()))
            .andThen(runMagazine(magazine, () -> -1)
                .withTimeout(0.5))
    );
    return runShooter.alongWith(runMagazine);
  }

  @Contract(pure = true)
  public static @NotNull Command shootAndDriveBackAuto(Drivetrain drivetrain, Magazine magazine, Shooter shooter) {
    return runShooter(shooter, () -> 1).withTimeout(3)
        .andThen(runShooter(shooter, () -> 1)
            .alongWith(runMagazine(magazine, () -> 1)))
        .withTimeout(5)
        .andThen(simpleArcadeDrive(drivetrain, () -> 1, () -> 0))
        .withTimeout(2);
  }
}
