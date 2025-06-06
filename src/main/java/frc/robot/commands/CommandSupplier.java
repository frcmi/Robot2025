package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CommandSupplier extends Command {
    private Command currentCommand;

    public CommandSupplier(Command commandToRun) {
        currentCommand = commandToRun;
    }
    public CommandSupplier() {}

    public void setCommand(Command c) {
        // this.m_requirements.clear();
        if (currentCommand != null) {
            currentCommand.end(true);
        }
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
    public boolean isFinished() {
        return currentCommand.isFinished();
    }    

    @Override
    public void end(boolean thing) {
        currentCommand.end(thing);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return currentCommand.getRequirements();
    }
}
