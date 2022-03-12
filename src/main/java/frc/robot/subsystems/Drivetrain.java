// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import si.uom.quantity.AngularSpeed;

import javax.measure.Quantity;
import javax.measure.quantity.Length;
import javax.measure.quantity.Speed;

public interface Drivetrain extends Subsystem {
  Quantity<Length> getLeftDistance();
  Quantity<Length> getRightDistance();
  Quantity<Length> getDistance();
  Quantity<Speed> getLeftVelocity();
  Quantity<Speed> getRightVelocity();
  Quantity<Speed> getLinearVelocity();
  Quantity<AngularSpeed> getAngularVelocity();
  boolean getLowGear();
  void setLowGear(boolean wantsLowGear);
  default void toggleLowGear() {
    setLowGear(!getLowGear());
  }
  void simpleArcadeDrive(double move, double turn);
  void velocityArcadeDrive(Quantity<Speed> linearVelocity, Quantity<AngularSpeed> angularVelocity);
}
