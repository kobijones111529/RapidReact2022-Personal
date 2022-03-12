package frc.robot.subsystems.physical;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class PhysicalIntake implements Intake {
  private final DoubleSolenoid extender = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.INTAKE_EXTEND_FORWARD_CHANNEL, Constants.INTAKE_EXTEND_REVERSE_CHANNEL);

  private final WPI_TalonSRX motor1 = new WPI_TalonSRX(Constants.INTAKE_MOTOR_1_ID);
  private final WPI_TalonSRX motor2 = new WPI_TalonSRX(Constants.INTAKE_MOTOR_2_ID);

  {
    motor1.configFactoryDefault();
    motor2.configFactoryDefault();

    motor1.setNeutralMode(Constants.INTAKE_MOTOR_NEUTRAL_MODE);
    motor2.setNeutralMode(Constants.INTAKE_MOTOR_NEUTRAL_MODE);

    motor2.follow(motor1);

    motor1.setInverted(InvertType.None);
    motor2.setInverted(InvertType.FollowMaster);
  }

  @Override
  public boolean getExtended() {
    return extender.get() == Constants.INTAKE_EXTENDED_VALUE;
  }

  @Override
  public void setExtended(boolean wantsExtended) {
    DoubleSolenoid.Value retractedValue = Constants.INTAKE_EXTENDED_VALUE == DoubleSolenoid.Value.kForward ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward;
    extender.set(wantsExtended ? Constants.INTAKE_EXTENDED_VALUE : retractedValue);
  }

  @Override
  public void toggleExtended() {
    extender.toggle();
  }

  @Override
  public double getOutput() {
    return motor1.get();
  }

  @Override
  public void setOutput(double output) {
    motor1.set(ControlMode.PercentOutput, output);
  }
}
