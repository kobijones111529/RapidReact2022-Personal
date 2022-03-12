package frc.robot.subsystems.dummy;

import frc.robot.subsystems.Shooter;
import si.uom.quantity.AngularSpeed;
import systems.uom.common.USCustomary;
import tech.units.indriya.quantity.Quantities;

import javax.measure.Quantity;

public class DummyShooter implements Shooter {
  @Override
  public Quantity<AngularSpeed> getSpeed() {
    return Quantities.getQuantity(0, USCustomary.REVOLUTION_PER_MINUTE);
  }

  @Override
  public void setOutput(double output) {

  }

  @Override
  public void setSpeed(Quantity<AngularSpeed> speed) {

  }

  @Override
  public void updatePID() {

  }

  @Override
  public void updatePID(Quantity<AngularSpeed> setpoint) {

  }
}
