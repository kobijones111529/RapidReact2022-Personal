package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.jetbrains.annotations.NotNull;

public class RepeatCommand extends CommandBase {
  protected final Command command;

  public RepeatCommand(@NotNull Command command) {
    this.command = command;
    m_requirements.addAll(command.getRequirements());
  }

  @Override
  public void initialize() {
    command.initialize();
  }

  @Override
  public void execute() {
    if (!command.isFinished()) {
      command.execute();
    }
    else {
      command.end(false);
      command.initialize();
    }
  }

  @Override
  public void end(boolean interrupted) {
    command.end(interrupted);
  }

  @Override
  public boolean runsWhenDisabled() {
    return command.runsWhenDisabled();
  }
}
