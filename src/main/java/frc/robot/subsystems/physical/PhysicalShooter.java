// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.physical;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;
import frc.robot.CustomUnits;
import frc.robot.subsystems.Shooter;
import si.uom.quantity.AngularSpeed;
import systems.uom.common.USCustomary;
import tech.units.indriya.quantity.Quantities;

import javax.measure.Quantity;

public class PhysicalShooter implements Shooter {
  private final WPI_TalonFX motor = new WPI_TalonFX(Constants.SHOOTER_MOTOR_ID);

  private final SimpleMotorFeedforward flywheelFeedforward;
  private final PIDController flywheelPID;
  private boolean pidUpdated = false;

  {
    motor.configFactoryDefault();
    motor.setInverted(InvertType.None);

    flywheelFeedforward = new SimpleMotorFeedforward(Constants.SHOOTER_FEEDFORWARD_S, Constants.SHOOTER_FEEDFORWARD_V, Constants.SHOOTER_FEEDFORWARD_A);
    flywheelPID = new PIDController(Constants.SHOOTER_PID_P, Constants.SHOOTER_PID_I, Constants.SHOOTER_PID_D);
  }

  @Override
  public void periodic() {
    if (!pidUpdated) {
      flywheelPID.reset();
    }
    pidUpdated = false;
  }

  @Override
  public Quantity<AngularSpeed> getSpeed() {
    double revolutionsPer100MS = motor.getSelectedSensorVelocity() / Constants.SHOOTER_ENCODER_RESOLUTION / Constants.SHOOTER_ENCODER_TO_OUTPUT_RATIO;
    return Quantities.getQuantity(revolutionsPer100MS, CustomUnits.REVOLUTION_PER_100_MILLISECOND).to(USCustomary.REVOLUTION_PER_MINUTE);
  }

  @Override
  public void setOutput(double output) {
    motor.set(ControlMode.PercentOutput, output);
  }

  @Override
  public void setSpeed(Quantity<AngularSpeed> speed) {
    double voltage = flywheelFeedforward.calculate(speed.to(USCustomary.REVOLUTION_PER_MINUTE).getValue().doubleValue());
    motor.setVoltage(voltage);
  }

  @Override
  public void updatePID() {
    double feedforward = flywheelFeedforward.calculate(flywheelPID.getSetpoint());
    double pid = flywheelPID.calculate(getSpeed().to(USCustomary.REVOLUTION_PER_MINUTE).getValue().doubleValue());
    motor.setVoltage(feedforward + pid);
  }

  @Override
  public void updatePID(Quantity<AngularSpeed> setpoint) {
    flywheelPID.setSetpoint(setpoint.to(USCustomary.REVOLUTION_PER_MINUTE).getValue().doubleValue());
    updatePID();
  }
}
