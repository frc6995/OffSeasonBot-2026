package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

/**
 * TEMPORARY - first-enable hitch diagnosis. Remove once the ~0.5s stall on the first autonomous
 * enable after a code restart has been located.
 *
 * <p>The CommandScheduler's own overrun printout only names top-level commands, so a stall
 * anywhere inside a composed auto shows up as one opaque number for the whole group. Wrapping
 * the pieces of a group with {@link #timed} times each piece's initialize()/execute()/
 * isFinished()/end() individually and prints to the console (RioLog / netconsole):
 *
 * <ul>
 *   <li>the first call of each phase, always - so cold first-run cost is visible even when
 *       it's small, and a clean second enable can be compared against it
 *   <li>any later call slower than {@link #kReportThresholdMs}
 * </ul>
 *
 * <p>The wrapper preserves the wrapped command's requirements, name and interruption behavior
 * (see {@link WrapperCommand}), so it doesn't change how the auto runs.
 */
public final class CommandTiming {
    private static final double kReportThresholdMs = 5.0;

    private CommandTiming() {}

    public static Command timed(String label, Command command) {
        return new Timed(label, command);
    }

    private static final class Timed extends WrapperCommand {
        private final String label;
        private boolean firstInitialize = true;
        private boolean firstExecute = true;
        private boolean firstIsFinished = true;
        private boolean firstEnd = true;

        Timed(String label, Command command) {
            super(command);
            this.label = label;
        }

        @Override
        public void initialize() {
            long start = System.nanoTime();
            super.initialize();
            report("initialize", start, firstInitialize);
            firstInitialize = false;
        }

        @Override
        public void execute() {
            long start = System.nanoTime();
            super.execute();
            report("execute", start, firstExecute);
            firstExecute = false;
        }

        @Override
        public boolean isFinished() {
            long start = System.nanoTime();
            boolean finished = super.isFinished();
            report("isFinished", start, firstIsFinished);
            firstIsFinished = false;
            return finished;
        }

        @Override
        public void end(boolean interrupted) {
            long start = System.nanoTime();
            super.end(interrupted);
            report("end", start, firstEnd);
            firstEnd = false;
        }

        // Measured before any string building, so printing never counts against the phase.
        // Plain concatenation rather than String.format: Formatter is itself cold on first use.
        private void report(String phase, long startNanos, boolean first) {
            double ms = (System.nanoTime() - startNanos) / 1e6;
            if (first || ms > kReportThresholdMs) {
                System.out.println("[CmdTiming] t=" + Math.round(Timer.getFPGATimestamp() * 1000) / 1000.0
                        + " " + label + "." + phase + "(): " + Math.round(ms * 100) / 100.0 + " ms"
                        + (first ? " (first)" : ""));
            }
        }
    }
}
