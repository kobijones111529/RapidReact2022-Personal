// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.utils.InterpolatingQuantity;
import frc.robot.utils.InterpolatingTreeMap;
import si.uom.SI;
import si.uom.quantity.AngularAcceleration;
import si.uom.quantity.AngularSpeed;
import tech.units.indriya.quantity.Quantities;

import javax.measure.Quantity;
import javax.measure.Unit;
import javax.measure.quantity.*;

import systems.uom.common.USCustomary;

public class Constants {
  public static final Quantity<Acceleration> GRAVITY = Quantities.getQuantity(-9.8, SI.METRE_PER_SQUARE_SECOND);

  // Field
  public static final Quantity<Length> HIGH_GOAL_HEIGHT = Quantities.getQuantity(0, SI.METRE); // TODO look up value

  // Spacial position/orientation
  // Relative to robot
  public static final Quantity<Length> LIMELIGHT_POS_X = Quantities.getQuantity(0, SI.METRE);
  public static final Quantity<Length> LIMELIGHT_POS_Z = Quantities.getQuantity(0, SI.METRE);
  public static final Quantity<Length> LIMELIGHT_HEIGHT = Quantities.getQuantity(0, SI.METRE);
  public static final Quantity<Angle> LIMELIGHT_MOUNT_ANGLE = Quantities.getQuantity(0, SI.RADIAN);

  // Robot
  public static final double INTAKE_MAX_OUTPUT = 1;
  public static final double MAGAZINE_MAX_OUTPUT = 1;
  public static final double SHOOTER_MAX_OUTPUT = 1;

  // Control board
  public static final int XBOX_PORT = 0;
  public static final int EXTREME_PORT = 1;
  public static final int BUTTON_BOX_PORT = 2;

  // Limelight
  public static final String LIMELIGHT_TABLE_NAME = "limelight";

  // CAN
  public static final int DRIVETRAIN_DRIVE_LEFT_FRONT_ID = 0;
  public static final int DRIVETRAIN_DRIVE_LEFT_BACK_ID = 1;
  public static final int DRIVETRAIN_DRIVE_RIGHT_FRONT_ID = 2;
  public static final int DRIVETRAIN_DRIVE_RIGHT_BACK_ID = 3;
  public static final int INTAKE_MOTOR_1_ID = 4;
  public static final int INTAKE_MOTOR_2_ID = 5;
  public static final int MAGAZINE_MOTOR_ID = 6;
  public static final int SHOOTER_MOTOR_ID = 7;

  // DIO
  public static final int DRIVETRAIN_ENCODER_LEFT_CHANNEL_A = 0;
  public static final int DRIVETRAIN_ENCODER_LEFT_CHANNEL_B = 1;
  public static final int DRIVETRAIN_ENCODER_RIGHT_CHANNEL_A = 2;
  public static final int DRIVETRAIN_ENCODER_RIGHT_CHANNEL_B = 3;

  // PCM
  public static final int DRIVETRAIN_SHIFT_FORWARD_CHANNEL = 0;
  public static final int DRIVETRAIN_SHIFT_REVERSE_CHANNEL = 1;
  public static final int INTAKE_EXTEND_FORWARD_CHANNEL = 4;
  public static final int INTAKE_EXTEND_REVERSE_CHANNEL = 5;

  // Drivetrain
  public static final Quantity<Length> DRIVETRAIN_TRACK_WIDTH = Quantities.getQuantity(60, USCustomary.INCH);
  public static final Quantity<Length> DRIVETRAIN_WHEEL_DIAMETER = Quantities.getQuantity(6, USCustomary.INCH);
  public static final Quantity<Length> DRIVETRAIN_WHEEL_CIRCUMFERENCE = DRIVETRAIN_WHEEL_DIAMETER.multiply(Math.PI * 2);
  public static final int DRIVETRAIN_ENCODER_RESOLUTION = 256;
  public static final double DRIVETRAIN_ENCODER_TO_OUTPUT_RATIO = 1;
  public static final Quantity<Length> DRIVETRAIN_ENCODER_DISTANCE_PER_PULSE = DRIVETRAIN_WHEEL_CIRCUMFERENCE.multiply(1. / DRIVETRAIN_ENCODER_RESOLUTION / DRIVETRAIN_ENCODER_TO_OUTPUT_RATIO);
  public static final Unit<Length> DRIVETRAIN_ENCODER_DISTANCE_UNIT = SI.METRE;
  public static final Unit<Speed> DRIVETRAIN_ENCODER_VELOCITY_UNIT = DRIVETRAIN_ENCODER_DISTANCE_UNIT.divide(SI.SECOND).asType(Speed.class);
  public static final double DRIVETRAIN_LEFT_FEEDFORWARD_S = 0;
  public static final double DRIVETRAIN_LEFT_FEEDFORWARD_V = 1;
  public static final double DRIVETRAIN_LEFT_FEEDFORWARD_A = 0;
  public static final double DRIVETRAIN_RIGHT_FEEDFORWARD_S = 0;
  public static final double DRIVETRAIN_RIGHT_FEEDFORWARD_V = 1;
  public static final double DRIVETRAIN_RIGHT_FEEDFORWARD_A = 0;
  public static final DoubleSolenoid.Value DRIVETRAIN_LOW_GEAR_VALUE = DoubleSolenoid.Value.kForward;

  public static final Quantity<Frequency> DRIVETRAIN_MOVE_RATE_LIMIT = Quantities.getQuantity(0.2, SI.HERTZ);
  public static final Quantity<Frequency> DRIVETRAIN_TURN_RATE_LIMIT = Quantities.getQuantity(0.2, SI.HERTZ);
  public static final Quantity<Acceleration> DRIVETRAIN_LINEAR_ACCELERATION_LIMIT = Quantities.getQuantity(0.2, SI.METRE_PER_SQUARE_SECOND);
  public static final Quantity<AngularAcceleration> DRIVETRAIN_ANGULAR_ACCELERATION_LIMIT = Quantities.getQuantity(0.2, SI.RADIAN_PER_SQUARE_SECOND);

  // Intake
  public static final DoubleSolenoid.Value INTAKE_EXTENDED_VALUE = DoubleSolenoid.Value.kForward;
  public static final NeutralMode INTAKE_MOTOR_NEUTRAL_MODE = NeutralMode.Brake;

  // Magazine
  public static final NeutralMode MAGAZINE_MOTOR_NEUTRAL_MODE = NeutralMode.Brake;

  // Shooter
  public static final int SHOOTER_ENCODER_RESOLUTION = 2048;
  public static final double SHOOTER_ENCODER_TO_OUTPUT_RATIO = 1;
  public static final double SHOOTER_FEEDFORWARD_S = 0;
  public static final double SHOOTER_FEEDFORWARD_V = 1;
  public static final double SHOOTER_FEEDFORWARD_A = 0;
  public static final double SHOOTER_PID_P = 0.01;
  public static final double SHOOTER_PID_I = 0;
  public static final double SHOOTER_PID_D = 0;
  public static final Quantity<Length> SHOOTER_MIN_DISTANCE = Quantities.getQuantity(0, SI.METRE);
  public static final Quantity<Length> SHOOTER_MAX_DISTANCE = Quantities.getQuantity(Double.POSITIVE_INFINITY, SI.METRE);
  public static final Quantity<AngularSpeed> SHOOTER_SPEED_ERROR_TOLERANCE = Quantities.getQuantity(100, USCustomary.REVOLUTION_PER_MINUTE);
  public static final Quantity<Angle> SHOOTER_ANGLE_ERROR_TOLERANCE = Quantities.getQuantity(5, USCustomary.DEGREE_ANGLE);
  public static final InterpolatingTreeMap<InterpolatingQuantity<Length>, InterpolatingQuantity<AngularSpeed>> SHOOTER_DISTANCE_SPEED_MAP = new InterpolatingTreeMap<>();
  static {
    SHOOTER_DISTANCE_SPEED_MAP.put(new InterpolatingQuantity<>(Quantities.getQuantity(0, SI.METRE)), new InterpolatingQuantity<>(Quantities.getQuantity(0, USCustomary.REVOLUTION_PER_MINUTE)));
  }

  private Constants() { }
}
