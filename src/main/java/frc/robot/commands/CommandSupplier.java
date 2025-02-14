package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Set;

public class CommandSupplier extends Command {
  private Command currentCommand;

  public CommandSupplier(Command commandToRun) {
    currentCommand = commandToRun;
  }

  public CommandSupplier() {}

  public void setCommand(Command c) {
    currentCommand.end(true);
    currentCommand = c;
  }

  @Override
  public void execute() {
    currentCommand.execute();
  }

  @Override
  public String getName() {
    return currentCommand.getName();
  }

  @Override
  public void initialize() {
    currentCommand.initialize();
  }

  @Override
  public Set<Subsystem> getRequirements() {
    return currentCommand.getRequirements();
  }

  @Override
  public boolean isFinished() {
    return currentCommand.isFinished();
  }
}
