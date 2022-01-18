// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AccelerationLimitedVelocityArcadeDrive extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final DoubleSupplier m_xSpeedSupplier;
  private final DoubleSupplier m_zRotationSupplier;
  private final SlewRateLimiter m_accelerationLimiter;
  private final SlewRateLimiter m_angularAccelerationLimiter;
  
  /**
   * Velocity controlled arcade drive with acceleration limiting
   * @param drivetrain Drivetrain subsystem
   * @param moveSupplier Move supplier in meters per second
   * @param turnSupplier Turn supplier in radians per second
   * @param maxAcceleration Max acceleration in m/s/s
   * @param maxAngularAcceleration Max angular acceleration in radians/s/s
   */
  public AccelerationLimitedVelocityArcadeDrive(final Drivetrain drivetrain, final DoubleSupplier xSpeedSupplier, final DoubleSupplier zRotationSupplier, double maxAcceleration, double maxAngularAcceleration) {
    m_drivetrain = drivetrain;
    m_xSpeedSupplier = xSpeedSupplier;
    m_zRotationSupplier = zRotationSupplier;
    m_accelerationLimiter = new SlewRateLimiter(maxAcceleration);
    m_angularAccelerationLimiter = new SlewRateLimiter(maxAngularAcceleration);
  
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = m_accelerationLimiter.calculate(m_xSpeedSupplier.getAsDouble());
    double zRotation = m_angularAccelerationLimiter.calculate(m_zRotationSupplier.getAsDouble());
    m_drivetrain.velocityArcadeDrive(xSpeed, zRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
