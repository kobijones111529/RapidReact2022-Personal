package frc.robot.subsystems.physical;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;
import frc.robot.subsystems.Magazine;

public class PhysicalMagazine implements Magazine {
  private final WPI_TalonSRX motor = new WPI_TalonSRX(Constants.MAGAZINE_MOTOR_ID);

  {
    motor.configFactoryDefault();
    motor.setInverted(InvertType.None);
    motor.setNeutralMode(Constants.MAGAZINE_MOTOR_NEUTRAL_MODE);
  }

  @Override
  public double getOutput() {
    return motor.get();
  }

  @Override
  public void setOutput(double output) {
    motor.set(ControlMode.PercentOutput, output);
  }
}
