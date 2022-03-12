package frc.robot.subsystems.dummy;

import frc.robot.subsystems.Magazine;

public class DummyMagazine implements Magazine {
  private double output = 0;

  @Override
  public double getOutput() {
    return output;
  }

  @Override
  public void setOutput(double output) {
    this.output = output;
  }
}
