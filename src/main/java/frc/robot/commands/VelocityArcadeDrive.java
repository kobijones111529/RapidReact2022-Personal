// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import si.uom.SI;
import si.uom.quantity.AngularSpeed;
import tech.units.indriya.quantity.Quantities;

import javax.measure.Quantity;
import javax.measure.quantity.Speed;

public class VelocityArcadeDrive extends CommandBase {
  private final Drivetrain drivetrain;
  private final Supplier<Quantity<Speed>> linearVelocitySupplier;
  private final Supplier<Quantity<AngularSpeed>> angularVelocitySupplier;

  /**
   * Velocity based arcade drive
   * @param drivetrain Drivetrain subsystem
   * @param linearVelocitySupplier The desired velocity in meters per second
   * @param angularVelocitySupplier The desired angular velocity in radians per second
   */
  public VelocityArcadeDrive(Drivetrain drivetrain, Supplier<Quantity<Speed>> linearVelocitySupplier, Supplier<Quantity<AngularSpeed>> angularVelocitySupplier) {
    this.drivetrain = drivetrain;
    this.linearVelocitySupplier = linearVelocitySupplier;
    this.angularVelocitySupplier = angularVelocitySupplier;

    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.velocityArcadeDrive(linearVelocitySupplier.get(), angularVelocitySupplier.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      drivetrain.velocityArcadeDrive(Quantities.getQuantity(0, SI.METRE_PER_SECOND), Quantities.getQuantity(0, SI.RADIAN_PER_SECOND));
    }
  }
}
