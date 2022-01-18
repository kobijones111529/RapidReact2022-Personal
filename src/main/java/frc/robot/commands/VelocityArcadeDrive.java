// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class VelocityArcadeDrive extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final DoubleSupplier m_xSpeedSupplier, m_zRotationSupplier;

  /**
   * Velocity based arcade drive
   * @param drivetrain Drivetrain subsystem
   * @param xSpeedSupplier The desired velocity in meters per second
   * @param zRotationSupplier The desired angular velocity in radians per second
   */
  public VelocityArcadeDrive(final Drivetrain drivetrain, final DoubleSupplier xSpeedSupplier, final DoubleSupplier zRotationSupplier) {
    m_drivetrain = drivetrain;
    m_xSpeedSupplier = xSpeedSupplier;
    m_zRotationSupplier = zRotationSupplier;

    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.velocityArcadeDrive(m_xSpeedSupplier.getAsDouble(), m_zRotationSupplier.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.velocityArcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
