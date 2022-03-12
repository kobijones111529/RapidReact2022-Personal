package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Magazine extends Subsystem {
  double getOutput();
  void setOutput(double output);
}
