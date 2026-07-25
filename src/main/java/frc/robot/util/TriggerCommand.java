package frc.robot.util;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TriggerCommand extends Command {
    private EventLoop m_eventLoop = new EventLoop();

    private ArrayList<Command> m_commands = new ArrayList<Command>();

    private TriggerCommand(Runnable toRun) {
        bind(toRun, () -> true);
    }

    private TriggerCommand(Command toRun) {
        bind(toRun, () -> true);
    }

    @Override
    public void execute() {
        super.execute();
        m_eventLoop.poll();
    }

    public static TriggerCommand create(Runnable toRun) {
        return new TriggerCommand(toRun);
    }

    public static TriggerCommand create(Command toRun) {
        return new TriggerCommand(toRun);
    }

    public TriggerCommand bind(Runnable toRun, BooleanSupplier booleanSupplier) {
        m_eventLoop.bind(
            new Runnable() {
                @Override
                public void run() {
                    if(booleanSupplier.getAsBoolean()) {
                        toRun.run();
                    }
                }
            }
        );
        return this;
    }

    public TriggerCommand bind(Command command, BooleanSupplier booleanSupplier) {
        return bind(command, booleanSupplier, false);
    }

    public TriggerCommand bind(Command command, BooleanSupplier booleanSupplier, boolean interrupt) {
        if(!interrupt && !m_commands.contains(command)) {
            for(Subsystem requirement : command.getRequirements()) {
                if(getRequirements().contains(requirement)) 
                    throw new RuntimeException("Multiple Commands in a TriggerCommand cannot require the same subsystem unless interrupt is enabled.");
            }
        }
        m_eventLoop.bind(
            new Runnable() {
                public void run() {
                    if(booleanSupplier.getAsBoolean()) {
                        if(interrupt) {
                            Set<Subsystem> reqs = command.getRequirements();
                            for(Command scheduledCommand : m_commands) {
                                if(scheduledCommand != command && scheduledCommand.getRequirements().stream().anyMatch(reqs::contains)) {
                                    CommandScheduler.getInstance().cancel(scheduledCommand);
                                }
                            }
                        }
                        CommandScheduler.getInstance().schedule(command);
                    }
                }
            }
        );
        m_commands.add(command);
        return this;
    }

}