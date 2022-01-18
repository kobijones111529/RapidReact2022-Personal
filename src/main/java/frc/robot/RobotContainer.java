// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.RunFlywheel;
import frc.robot.commands.VelocityArcadeDrive;
import frc.robot.controlboard.JoystickCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private Drivetrain m_drivetrain;
  private Shooter m_shooter;

  private Command m_autoCommand;

  private final Map<JoystickCommand, DoubleSupplier> m_joystickMap = new HashMap<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();

    Drivetrain.HardwareMap drivetrainMap = new Drivetrain.HardwareMap();
    drivetrainMap.driveLeftFrontID = 0;
    drivetrainMap.driveLeftBackID = 1;
    drivetrainMap.driveRightFrontID = 2;
    drivetrainMap.driveRightBackID = 3;
    drivetrainMap.driveLeftEncoderChannelA = 0;
    drivetrainMap.driveLeftEncoderChannelB = 1;
    drivetrainMap.driveRightEncoderChannelA = 2;
    drivetrainMap.driveRightEncoderChannelB = 3;
    drivetrainMap.pigeonID = 5;
    m_drivetrain = new Drivetrain(drivetrainMap);

    Shooter.HardwareMap shooterMap = new Shooter.HardwareMap();
    shooterMap.flywheelID = 0;
    shooterMap.flywheelEncoderChannelA = 0;
    shooterMap.flywheelEncoderChannelB = 1;
    m_shooter = new Shooter(shooterMap);

    m_drivetrain.setDefaultCommand(new VelocityArcadeDrive(m_drivetrain, m_joystickMap.get(JoystickCommand.MOVE), m_joystickMap.get(JoystickCommand.TURN)));
    m_shooter.setDefaultCommand(new RunFlywheel(m_shooter, () -> 0));

    m_autoCommand = new AutoCommand();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_joystickMap.put(JoystickCommand.MOVE, () -> {
      return 0;
    });
    m_joystickMap.put(JoystickCommand.TURN, () -> {
      return 0;
    });
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoCommand;
  }
}
