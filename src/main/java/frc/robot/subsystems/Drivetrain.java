// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  public static class HardwareMap {
    public int driveLeftFrontID;
    public int driveLeftBackID;
    public int driveRightFrontID;
    public int driveRightBackID;
    public int driveLeftEncoderChannelA;
    public int driveLeftEncoderChannelB;
    public int driveRightEncoderChannelA;
    public int driveRightEncoderChannelB;
    public int pigeonID;
  }

  // TODO: TEMP: find actual values
  public final double MAX_SPEED_HIGH_GEAR = 10; // Meters per second
  public final double MAX_SPEED_LOW_GEAR = 5; // Meters per second

  private final double TRACK_WIDTH_METERS = 1; // Meters
  private final double LEFT_KS = 0;
  private final double LEFT_KV = 0;
  private final double LEFT_KA = 0;
  private final double RIGHT_KS = 0;
  private final double RIGHT_KV = 0;
  private final double RIGHT_KA = 0;

  private final WPI_TalonFX m_leftDriveMaster;
  private final WPI_TalonFX m_leftDriveFollower;
  private final WPI_TalonFX m_rightDriveMaster;
  private final WPI_TalonFX m_rightDriveFollower;

  private final Encoder m_leftDriveEncoder;
  private final Encoder m_rightDriveEncoder;

  private final Pigeon2 m_pigeon;

  private final DifferentialDrive m_differentialDrive;

  private final DifferentialDriveKinematics m_driveKinematics;
  private final DifferentialDriveOdometry m_odometry;
  private Pose2d m_pose;

  private final SimpleMotorFeedforward m_leftDriveFeedforward;
  private final SimpleMotorFeedforward m_rightDriveFeedforward;

  public Drivetrain(HardwareMap map) {
    m_leftDriveMaster = new WPI_TalonFX(map.driveLeftBackID);
    m_leftDriveFollower = new WPI_TalonFX(map.driveLeftBackID);
    m_rightDriveMaster = new WPI_TalonFX(map.driveRightFrontID);
    m_rightDriveFollower = new WPI_TalonFX(map.driveRightBackID);

    m_leftDriveEncoder = new Encoder(map.driveLeftEncoderChannelA, map.driveLeftEncoderChannelB, false);
    m_rightDriveEncoder = new Encoder(map.driveRightEncoderChannelA, map.driveRightEncoderChannelB, false);

    m_pigeon = new Pigeon2(map.pigeonID);

    // Set to factory defaults to prevent unexpected behavior
    // Desired behavior can be configured programmatically
    m_leftDriveMaster.configFactoryDefault();
    m_leftDriveFollower.configFactoryDefault();
    m_rightDriveMaster.configFactoryDefault();
    m_rightDriveFollower.configFactoryDefault();

    m_pigeon.configFactoryDefault();

    m_leftDriveFollower.follow(m_leftDriveMaster);
    m_rightDriveFollower.follow(m_rightDriveMaster);

    // Set motor output direction
    m_leftDriveMaster.setInverted(InvertType.InvertMotorOutput);
    m_leftDriveFollower.setInverted(InvertType.FollowMaster);
    m_rightDriveMaster.setInverted(InvertType.None);
    m_rightDriveFollower.setInverted(InvertType.FollowMaster);

    m_differentialDrive = new DifferentialDrive(m_leftDriveMaster, m_rightDriveMaster);
  
    m_pose = new Pose2d(0, 0, new Rotation2d(0));
    m_driveKinematics = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);
    // TODO: set odometry initialization values
    m_odometry = new DifferentialDriveOdometry(new Rotation2d(getGyroAngle()), m_pose);

    // TODO: set kinematics values
    m_leftDriveFeedforward = new SimpleMotorFeedforward(LEFT_KS, LEFT_KV, LEFT_KA);
    m_rightDriveFeedforward = new SimpleMotorFeedforward(RIGHT_KS, RIGHT_KV, RIGHT_KA);
  }

  @Override
  public void periodic() {
    m_pose = m_odometry.update(new Rotation2d(getGyroAngle()), getLeftEncoderDistance(), getRightEncoderDistance());
  }

  public void simpleArcadeDrive(double xSpeed, double zRotation) {
    m_differentialDrive.arcadeDrive(xSpeed, zRotation);
  }

  /**
   * Set linear and angular velocity setpoints for the drivetrain
   * @param xSpeed The desired velocity in meters per second
   * @param zRotation The desired angular velocity (clockwise) in radians per second
   */
  public void velocityArcadeDrive(double xSpeed, double zRotation) {
    velocityArcadeDrive(new ChassisSpeeds(xSpeed, 0, zRotation));
  }

  /**
   * Set chassis speeds set point
   * @param chassisSpeeds The desired chassis speeds in meters and radians per second
   */
  public void velocityArcadeDrive(final ChassisSpeeds chassisSpeeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = m_driveKinematics.toWheelSpeeds(chassisSpeeds);
    double leftOutputVoltage = m_leftDriveFeedforward.calculate(wheelSpeeds.leftMetersPerSecond);
    double rightOutputVoltage = m_rightDriveFeedforward.calculate(wheelSpeeds.rightMetersPerSecond);
    m_leftDriveMaster.setVoltage(leftOutputVoltage);
    m_rightDriveMaster.setVoltage(rightOutputVoltage);
  }

  public double getLeftEncoderDistance() {
    return m_leftDriveEncoder.getDistance();
  }

  public double getRightEncoderDistance() {
    return m_rightDriveEncoder.getDistance();
  }

  public double getGyroAngle() {
    return m_pigeon.getYaw();
  }
}
