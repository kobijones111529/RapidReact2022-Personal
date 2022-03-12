// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import si.uom.quantity.AngularSpeed;

import javax.measure.Quantity;
import javax.measure.quantity.Length;

public interface Shooter extends Subsystem {
  Quantity<AngularSpeed> getSpeed();
  /**
   * Set flywheel percentage output
   * @param output Motor output [-1 to +1]
   */
  void setOutput(double output);
  /**
   * Set flywheel rpm
   * @param speed Desired speed
   */
  void setSpeed(Quantity<AngularSpeed> speed);

  /**
   * Update flywheel velocity PID using existing setpoint
   * <p/>
   * PID will be disabled if not updated every frame
   */
  void updatePID();

  /**
   * Update flywheel velocity PID using new setpoint
   * <p/>
   * PID will be disabled if not updated every frame
   * @param setpoint PID setpoint
   */
  void updatePID(Quantity<AngularSpeed> setpoint);
}
