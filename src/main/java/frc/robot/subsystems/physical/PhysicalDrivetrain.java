// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.physical;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import si.uom.SI;
import si.uom.quantity.AngularSpeed;
import tech.units.indriya.quantity.Quantities;
import tech.units.indriya.unit.Units;

import javax.measure.Quantity;
import javax.measure.quantity.Length;
import javax.measure.quantity.Speed;

public class PhysicalDrivetrain implements Drivetrain {
  NetworkTable table;

  NetworkTableEntry maxSpeedHighGearEntry;
  NetworkTableEntry maxSpeedLowGearEntry;
  NetworkTableEntry trackWidthMetersEntry;
  NetworkTableEntry leftKSEntry;
  NetworkTableEntry leftKVEntry;
  NetworkTableEntry leftKAEntry;
  NetworkTableEntry rightKSEntry;
  NetworkTableEntry rightKVEntry;
  NetworkTableEntry rightKAEntry;

  NetworkTableEntry leftDriveDistanceEntry;
  NetworkTableEntry rightDriveDistanceEntry;
  NetworkTableEntry leftDriveVelocityEntry;
  NetworkTableEntry rightDriveVelocityEntry;
  NetworkTableEntry gyroAngleEntry;

  private final WPI_TalonFX leftDriveMaster;
  private final WPI_TalonFX rightDriveMaster;

  private final DoubleSolenoid shifter;

  private final Encoder leftDriveEncoder;
  private final Encoder rightDriveEncoder;

  private final DifferentialDrive differentialDrive;

  private final DifferentialDriveKinematics driveKinematics;

  private final SimpleMotorFeedforward leftDriveFeedforward;
  private final SimpleMotorFeedforward rightDriveFeedforward;

  private final SlewRateLimiter moveLimiter = new SlewRateLimiter(Constants.DRIVETRAIN_MOVE_RATE_LIMIT.to(SI.HERTZ).getValue().doubleValue());
  private final SlewRateLimiter turnLimiter = new SlewRateLimiter(Constants.DRIVETRAIN_TURN_RATE_LIMIT.to(SI.HERTZ).getValue().doubleValue());
  private final SlewRateLimiter linearAccelerationLimiter = new SlewRateLimiter(Constants.DRIVETRAIN_LINEAR_ACCELERATION_LIMIT.to(SI.METRE_PER_SQUARE_SECOND).getValue().doubleValue());
  private final SlewRateLimiter angularAccelerationLimiter = new SlewRateLimiter(Constants.DRIVETRAIN_ANGULAR_ACCELERATION_LIMIT.to(SI.RADIAN_PER_SQUARE_SECOND).getValue().doubleValue());

  public PhysicalDrivetrain() {
    table = NetworkTableInstance.getDefault().getTable("Drivetrain");

    gyroAngleEntry = table.getEntry("Gyro enabled");
    maxSpeedHighGearEntry = table.getEntry("Max speed high gear");
    maxSpeedLowGearEntry = table.getEntry("Max speed low gear");
    trackWidthMetersEntry = table.getEntry("Track width (meters)");
    leftKSEntry = table.getEntry("Left kS");
    leftKVEntry = table.getEntry("Left kV");
    leftKAEntry = table.getEntry("Left kA");
    rightKSEntry = table.getEntry("Right kS");
    rightKVEntry = table.getEntry("Right kV");
    rightKAEntry = table.getEntry("Right kA");

    leftDriveDistanceEntry = table.getEntry("Left distance");
    rightDriveDistanceEntry = table.getEntry("Right distance");
    leftDriveVelocityEntry = table.getEntry("Left velocity");
    rightDriveVelocityEntry = table.getEntry("Right velocity");
    gyroAngleEntry = table.getEntry("Gyro angle");

    leftDriveMaster = new WPI_TalonFX(Constants.DRIVETRAIN_DRIVE_LEFT_FRONT_ID);
    WPI_TalonFX leftDriveFollower = new WPI_TalonFX(Constants.DRIVETRAIN_DRIVE_LEFT_BACK_ID);
    rightDriveMaster = new WPI_TalonFX(Constants.DRIVETRAIN_DRIVE_RIGHT_FRONT_ID);
    WPI_TalonFX rightDriveFollower = new WPI_TalonFX(Constants.DRIVETRAIN_DRIVE_RIGHT_BACK_ID);

    // Set to factory defaults to prevent unexpected behavior
    // Desired behavior can be configured programmatically
    leftDriveMaster.configFactoryDefault();
    leftDriveFollower.configFactoryDefault();
    rightDriveMaster.configFactoryDefault();
    rightDriveFollower.configFactoryDefault();

    leftDriveFollower.follow(leftDriveMaster);
    rightDriveFollower.follow(rightDriveMaster);

    leftDriveMaster.setInverted(InvertType.InvertMotorOutput);
    leftDriveFollower.setInverted(InvertType.FollowMaster);
    rightDriveMaster.setInverted(InvertType.None);
    rightDriveFollower.setInverted(InvertType.FollowMaster);

    shifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.DRIVETRAIN_SHIFT_FORWARD_CHANNEL, Constants.DRIVETRAIN_SHIFT_REVERSE_CHANNEL);

    leftDriveEncoder = new Encoder(Constants.DRIVETRAIN_ENCODER_LEFT_CHANNEL_A, Constants.DRIVETRAIN_ENCODER_LEFT_CHANNEL_B);
    rightDriveEncoder = new Encoder(Constants.DRIVETRAIN_ENCODER_RIGHT_CHANNEL_A, Constants.DRIVETRAIN_ENCODER_RIGHT_CHANNEL_B);

    leftDriveEncoder.setDistancePerPulse(Constants.DRIVETRAIN_ENCODER_DISTANCE_PER_PULSE.to(Constants.DRIVETRAIN_ENCODER_DISTANCE_UNIT).getValue().doubleValue());

    differentialDrive = new DifferentialDrive(leftDriveMaster, rightDriveMaster);

    driveKinematics = new DifferentialDriveKinematics(Constants.DRIVETRAIN_TRACK_WIDTH.to(Units.METRE).getValue().doubleValue());

    leftDriveFeedforward = new SimpleMotorFeedforward(Constants.DRIVETRAIN_LEFT_FEEDFORWARD_S, Constants.DRIVETRAIN_LEFT_FEEDFORWARD_V, Constants.DRIVETRAIN_LEFT_FEEDFORWARD_A);
    rightDriveFeedforward = new SimpleMotorFeedforward(Constants.DRIVETRAIN_RIGHT_FEEDFORWARD_S, Constants.DRIVETRAIN_RIGHT_FEEDFORWARD_V, Constants.DRIVETRAIN_RIGHT_FEEDFORWARD_A);
  }

  @Override
  public Quantity<Length> getLeftDistance() {
    return Quantities.getQuantity(leftDriveEncoder.getDistance(), Constants.DRIVETRAIN_ENCODER_DISTANCE_UNIT);
  }

  @Override
  public Quantity<Length> getRightDistance() {
    return Quantities.getQuantity(rightDriveEncoder.getDistance(), Constants.DRIVETRAIN_ENCODER_DISTANCE_UNIT);
  }

  @Override
  public Quantity<Length> getDistance() {
    return getLeftDistance().add(getRightDistance()).divide(2);
  }

  @Override
  public Quantity<Speed> getLeftVelocity() {
    return Quantities.getQuantity(leftDriveEncoder.getRate(), Constants.DRIVETRAIN_ENCODER_VELOCITY_UNIT);
  }

  @Override
  public Quantity<Speed> getRightVelocity() {
    return Quantities.getQuantity(rightDriveEncoder.getRate(), Constants.DRIVETRAIN_ENCODER_VELOCITY_UNIT);
  }

  @Override
  public Quantity<Speed> getLinearVelocity() {
    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(getLeftVelocity().to(SI.METRE_PER_SECOND).getValue().doubleValue(), getRightVelocity().to(SI.METRE_PER_SECOND).getValue().doubleValue());
    return Quantities.getQuantity(driveKinematics.toChassisSpeeds(wheelSpeeds).vxMetersPerSecond, SI.METRE_PER_SECOND);
  }

  @Override
  public Quantity<AngularSpeed> getAngularVelocity() {
    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(getLeftVelocity().to(SI.METRE_PER_SECOND).getValue().doubleValue(), getRightVelocity().to(SI.METRE_PER_SECOND).getValue().doubleValue());
    return Quantities.getQuantity(driveKinematics.toChassisSpeeds(wheelSpeeds).omegaRadiansPerSecond, SI.RADIAN_PER_SECOND);
  }

  @Override
  public boolean getLowGear() {
    return shifter.get() == Constants.DRIVETRAIN_LOW_GEAR_VALUE;
  }

  @Override
  public void setLowGear(boolean wantsLowGear) {
    DoubleSolenoid.Value lowGearValue = Constants.DRIVETRAIN_LOW_GEAR_VALUE;
    DoubleSolenoid.Value highGearValue = Constants.DRIVETRAIN_LOW_GEAR_VALUE == DoubleSolenoid.Value.kForward ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward;
    shifter.set(wantsLowGear ? lowGearValue : highGearValue);
  }

  @Override
  public void simpleArcadeDrive(double move, double turn) {
    move = moveLimiter.calculate(move);
    turn = turnLimiter.calculate(turn);
    differentialDrive.arcadeDrive(move, turn);
  }

  @Override
  public void velocityArcadeDrive(Quantity<Speed> linearVelocity, Quantity<AngularSpeed> angularVelocity) {
    double linearVelocityMetrePerSecond = linearVelocity.to(SI.METRE_PER_SECOND).getValue().doubleValue();
    double angularVelocityRadianPerSecond = angularVelocity.to(SI.RADIAN_PER_SECOND).getValue().doubleValue();
    linearVelocityMetrePerSecond = linearAccelerationLimiter.calculate(linearVelocityMetrePerSecond);
    angularVelocityRadianPerSecond = angularAccelerationLimiter.calculate(angularVelocityRadianPerSecond);
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(linearVelocityMetrePerSecond, 0, angularVelocityRadianPerSecond);
    DifferentialDriveWheelSpeeds wheelSpeeds = driveKinematics.toWheelSpeeds(chassisSpeeds);
    double leftOutputVoltage = leftDriveFeedforward.calculate(wheelSpeeds.leftMetersPerSecond);
    double rightOutputVoltage = rightDriveFeedforward.calculate(wheelSpeeds.rightMetersPerSecond);
    leftDriveMaster.setVoltage(leftOutputVoltage);
    rightDriveMaster.setVoltage(rightOutputVoltage);
  }
}
