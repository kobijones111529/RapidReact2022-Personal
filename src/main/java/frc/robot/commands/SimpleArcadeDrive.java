// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class SimpleArcadeDrive extends CommandBase {
  private final Drivetrain drivetrain;
  private final DoubleSupplier moveSupplier, turnSupplier;

  public SimpleArcadeDrive(final Drivetrain drivetrain, final DoubleSupplier moveSupplier, final DoubleSupplier turnSupplier) {
    this.drivetrain = drivetrain;
    this.moveSupplier = moveSupplier;
    this.turnSupplier = turnSupplier;

    addRequirements(this.drivetrain);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.simpleArcadeDrive(moveSupplier.getAsDouble(), turnSupplier.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      drivetrain.simpleArcadeDrive(0, 0);
    }
  }
}
