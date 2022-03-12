// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.dummy;

import frc.robot.subsystems.Drivetrain;
import si.uom.SI;
import si.uom.quantity.AngularSpeed;
import tech.units.indriya.quantity.Quantities;

import javax.measure.Quantity;
import javax.measure.quantity.Length;
import javax.measure.quantity.Speed;

public class DummyDrivetrain implements Drivetrain {
  @Override
  public Quantity<Length> getLeftDistance() {
    return Quantities.getQuantity(0, SI.METRE);
  }

  @Override
  public Quantity<Length> getRightDistance() {
    return Quantities.getQuantity(0, SI.METRE);
  }

  @Override
  public Quantity<Length> getDistance() {
    return Quantities.getQuantity(0, SI.METRE);
  }

  @Override
  public Quantity<Speed> getLeftVelocity() {
    return Quantities.getQuantity(0, SI.METRE_PER_SECOND);
  }

  @Override
  public Quantity<Speed> getRightVelocity() {
    return Quantities.getQuantity(0, SI.METRE_PER_SECOND);
  }

  @Override
  public Quantity<Speed> getLinearVelocity() {
    return Quantities.getQuantity(0, SI.METRE_PER_SECOND);
  }

  @Override
  public Quantity<AngularSpeed> getAngularVelocity() {
    return Quantities.getQuantity(0, SI.RADIAN_PER_SECOND);
  }

  @Override
  public boolean getLowGear() {
    return false;
  }

  @Override
  public void setLowGear(boolean wantsLowGear) {

  }

  @Override
  public void simpleArcadeDrive(double move, double turn) {

  }

  @Override
  public void velocityArcadeDrive(Quantity<Speed> linearVelocity, Quantity<AngularSpeed> angularVelocity) {

  }
}
