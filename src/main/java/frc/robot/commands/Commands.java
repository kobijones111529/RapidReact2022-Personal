package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

import javax.measure.Quantity;
import javax.measure.quantity.Angle;
import javax.measure.quantity.Length;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public final class Commands {
  private Commands() {

  }

  public static Command simpleArcadeDrive(Drivetrain drivetrain, DoubleSupplier move, DoubleSupplier turn) {
    return new SimpleArcadeDrive(drivetrain, move, turn);
  }

  public static Command toggleDriveShifter(Drivetrain drivetrain) {
    return new InstantCommand(drivetrain::toggleLowGear, drivetrain);
  }

  public static Command driveToTarget(Drivetrain drivetrain, Supplier<Quantity<Length>> distanceFromTarget, Supplier<Quantity<Angle>> angleFromTarget, Quantity<Length> distanceTolerance, Quantity<Angle> angleTolerance) {
    return new DriveToTarget(drivetrain, distanceFromTarget, angleFromTarget, distanceTolerance, angleTolerance);
  }

  public static Command toggleIntakeExtended(Intake intake) {
    return new InstantCommand(intake::toggleExtended, intake);
  }

  public static Command runIntake(Intake intake, DoubleSupplier speed) {
    return new RunCommand(() -> intake.setOutput(speed.getAsDouble()), intake);
  }

  public static Command runMagazine(Magazine magazine, DoubleSupplier speed) {
    return new RunCommand(() -> magazine.setOutput(speed.getAsDouble()), magazine);
  }

  public static Command runShooter(Shooter shooter, DoubleSupplier speed) {
    return new RunCommand(() -> shooter.setOutput(speed.getAsDouble()), shooter);
  }

  public static Command shootAndDriveBackAuto(Drivetrain drivetrain, Magazine magazine, Shooter shooter) {
    return runShooter(shooter, () -> 1).withTimeout(3)
        .andThen(runShooter(shooter, () -> 1)
            .alongWith(runMagazine(magazine, () -> 1)))
        .withTimeout(5)
        .andThen(simpleArcadeDrive(drivetrain, () -> 1, () -> 0))
        .withTimeout(2);
  }
}
