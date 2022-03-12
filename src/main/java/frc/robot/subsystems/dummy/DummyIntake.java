package frc.robot.subsystems.dummy;

import frc.robot.subsystems.Intake;

public class DummyIntake implements Intake {
  private boolean extended = false;
  private double motorOutput = 0.;

  @Override
  public boolean getExtended() {
    return extended;
  }

  @Override
  public void setExtended(boolean wantsExtended) {
    extended = wantsExtended;
  }

  @Override
  public void toggleExtended() {
    extended = !extended;
  }

  @Override
  public double getOutput() {
    return motorOutput;
  }

  @Override
  public void setOutput(double output) {
    motorOutput = output;
  }
}
