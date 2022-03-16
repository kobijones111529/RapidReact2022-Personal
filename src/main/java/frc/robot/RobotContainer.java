// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Commands;
import frc.robot.commands.SimpleArcadeDrive;
import frc.robot.io.ControlBoard;
import frc.robot.io.Input;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.dummy.DummyDrivetrain;
import frc.robot.subsystems.dummy.DummyIntake;
import frc.robot.subsystems.dummy.DummyMagazine;
import frc.robot.subsystems.dummy.DummyShooter;
import frc.robot.subsystems.physical.PhysicalDrivetrain;
import frc.robot.subsystems.physical.PhysicalIntake;
import frc.robot.subsystems.physical.PhysicalMagazine;
import frc.robot.subsystems.physical.PhysicalShooter;
import frc.robot.utils.LimelightUtil;
import si.uom.SI;
import tech.units.indriya.quantity.Quantities;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final boolean drivetrainEnabled = true;
  private final boolean intakeEnabled = true;
  private final boolean magazineEnabled = true;
  private final boolean shooterEnabled = true;

  private final Input input = new Input(new ControlBoard(Constants.XBOX_PORT, Constants.EXTREME_PORT, Constants.BUTTON_BOX_PORT));

  private final Drivetrain drivetrain = drivetrainEnabled ? new PhysicalDrivetrain() : new DummyDrivetrain();
  private final Intake intake = intakeEnabled ? new PhysicalIntake() : new DummyIntake();
  private final Magazine magazine = magazineEnabled ? new PhysicalMagazine() : new DummyMagazine();
  private final Shooter shooter = shooterEnabled ? new PhysicalShooter() : new DummyShooter();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrain.setDefaultCommand(new SimpleArcadeDrive(drivetrain, input::getMove, input::getTurn));

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    input.getToggleDriveShifterTrigger().whenActive(Commands.toggleDriveShifter(drivetrain));
    input.getAutoAlignToTargetTrigger().whileActiveContinuous(Commands.autoAlignToTarget(drivetrain, () -> LimelightUtil.getTargetXOffset(Constants.LIMELIGHT_TABLE_NAME).orElse(Quantities.getQuantity(0, SI.RADIAN))));
    input.getRunIntakeTrigger().whileActiveContinuous(Commands.runIntake(intake, () -> Constants.INTAKE_MAX_OUTPUT));
    input.getRunMagazineTrigger().whileActiveContinuous(Commands.runMagazine(magazine, () -> Constants.MAGAZINE_MAX_OUTPUT));
    input.getRunShooterTrigger().whileActiveContinuous(Commands.runShooter(shooter, () -> Constants.SHOOTER_MAX_OUTPUT));
    input.getAutoShootTrigger().whileActiveContinuous(Commands.autoShoot(
        magazine,
        shooter,
        () -> LimelightUtil.getDistanceToTarget(Constants.LIMELIGHT_TABLE_NAME, Constants.LIMELIGHT_HEIGHT, Constants.LIMELIGHT_MOUNT_ANGLE, Constants.HIGH_GOAL_HEIGHT),
        () -> LimelightUtil.getTargetXOffset(Constants.LIMELIGHT_TABLE_NAME)
    ));
  }

  public void writeNetworkTables() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
