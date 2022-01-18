// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AccelerationLimitedSimpleArcadeDrive extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final DoubleSupplier m_moveSupplier;
  private final DoubleSupplier m_turnSupplier;
  private final SlewRateLimiter m_accelerationLimiter;
  private final SlewRateLimiter m_angularAccelerationLimiter;
  
  /**
   * Simple arcade drive with acceleration limiting
   * @param drivetrain Drivetrain subsystem
   * @param moveSupplier Move supplier [-1 to +1]
   * @param turnSupplier Turn supplier [-1 to +1]
   * @param maxAcceleration Max acceleration in normalized units
   * @param maxAngularAcceleration Max angular acceleration in normalized units
   */
  public AccelerationLimitedSimpleArcadeDrive(final Drivetrain drivetrain, final DoubleSupplier moveSupplier, final DoubleSupplier turnSupplier, double maxAcceleration, double maxAngularAcceleration) {
    m_drivetrain = drivetrain;
    m_moveSupplier = moveSupplier;
    m_turnSupplier = turnSupplier;
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
    double move = m_accelerationLimiter.calculate(m_moveSupplier.getAsDouble());
    double turn = m_angularAccelerationLimiter.calculate(m_turnSupplier.getAsDouble());
    m_drivetrain.simpleArcadeDrive(move, turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.simpleArcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
