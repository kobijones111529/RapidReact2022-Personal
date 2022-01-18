// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  public static class HardwareMap {
    public int flywheelID;
    public int flywheelEncoderChannelA;
    public int flywheelEncoderChannelB;
  }

  private final NetworkTable m_table;
  private final NetworkTableEntry m_rpmEntry;

  // TODO: TEMP: find actual values
  public final double ENCODER_TO_OUTPUT_RATIO = 1; // Encoder rotations to flywheel rotations
  public final double FW_KS = 0;
  public final double FW_KV = 0;
  public final double FW_KA = 0;
  public final double FW_KP = 0;
  public final double FW_KI = 0;
  public final double FW_KD = 0;

  private final WPI_TalonFX m_flywheel;

  private final Encoder m_flywheelEncoder;

  private final SimpleMotorFeedforward m_flywheelFeedforward;
  private final PIDController m_flywheelPID;

  public Shooter(final HardwareMap map) {
    m_table = NetworkTableInstance.getDefault().getTable("Shooter");
    m_rpmEntry = m_table.getEntry("RPM");


    m_flywheel = new WPI_TalonFX(map.flywheelID);

    m_flywheel.configFactoryDefault();

    m_flywheel.setInverted(InvertType.None);

    m_flywheelEncoder = new Encoder(map.flywheelEncoderChannelA, map.flywheelEncoderChannelB, false);
    m_flywheelEncoder.setDistancePerPulse(1. / ENCODER_TO_OUTPUT_RATIO);

    m_flywheelFeedforward = new SimpleMotorFeedforward(FW_KS, FW_KV, FW_KA);
    m_flywheelPID = new PIDController(FW_KP, FW_KI, FW_KD);
  }

  @Override
  public void periodic() {
    m_rpmEntry.setDouble(getRPM());
  }

  /**
   * Set flywheel percentage output
   * @param output Motor output [-1 to +1]
   */
  public void set(double output) {
    m_flywheel.set(output);
  }

  /**
   * Set flywheel rpm
   * @param rpm Desired rpm
   */
  public void setRPM(double rpm) {
    double voltageFeedforward = m_flywheelFeedforward.calculate(rpm);
    double pid = m_flywheelPID.calculate(getRPM(), rpm);
    m_flywheel.setVoltage(voltageFeedforward + pid);
  }

  public double getRPM() {
    return m_flywheelEncoder.getRate();
  }
}
