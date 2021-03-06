// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class RunFlywheel extends CommandBase {
  private final Shooter m_shooter;
  private final DoubleSupplier m_rpmSupplier;

  public RunFlywheel(final Shooter shooter, final DoubleSupplier rpmSupplier) {
    m_shooter = shooter;
    m_rpmSupplier = rpmSupplier;

    addRequirements(m_shooter);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO do thing
  }
}
