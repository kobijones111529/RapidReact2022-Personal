package frc.robot.io;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Input {
  private final ControlBoard controlBoard;

  private final Trigger toggleDriveShifter = new Trigger(); // TODO set
  private final Trigger runIntake = new Trigger(); // TODO set
  private final Trigger runMagazine = new Trigger(); // TODO set
  private final Trigger runShooter = new Trigger(); // TODO set

  public Input(ControlBoard controlBoard) {
    this.controlBoard = controlBoard;
  }

  public double getMove() {
    return controlBoard.xbox.getLeftY();
  }
  public double getTurn() {
    return controlBoard.xbox.getRightX();
  }
  public Trigger getToggleDriveShifterTrigger() {
    return toggleDriveShifter;
  }
  public Trigger getRunIntakeTrigger() {
    return runIntake;
  }
  public Trigger getRunMagazineTrigger() {
    return runMagazine;
  }
  public Trigger getRunShooterTrigger() {
    return runShooter;
  }
}
