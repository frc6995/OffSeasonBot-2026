package frc.robot.util.logging;

import java.io.File;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.Superstructure;
import frc.robot.util.CtreUtil;

/**
 * Turns CTRE's hoot signal logging on and off from the dashboard, and adds the handful of values
 * the power dashboard needs that no CTRE device reports.
 *
 * <p>This is the whole on-robot side of power logging. Phoenix records every motor's supply current
 * and supply voltage into the hoot file as the frames arrive on the bus, in its own thread, so the
 * robot loop never reads or writes a current value (see {@link CtreUtil#setPowerSignalFrequency}).
 * What this class adds per loop is a few cheap reads and comparisons; a hoot write happens only when
 * one of those values actually changes, a few dozen times a match.
 *
 * <p>The toggle is {@code /SmartDashboard/Hoot Logging/Enabled}, so it shows up in Elastic without
 * setup. It is a persistent NetworkTables value: set it once and it survives reboots and redeploys,
 * which is what an A/B test of loop timing across several power cycles needs.
 * {@code /SmartDashboard/Hoot Logging/Active} reports whether a log is actually being written.
 *
 * <p>Phoenix already logs {@code RobotMode} and {@code RobotEnable} itself. Custom signals written
 * here (names as they appear after export):
 * <ul>
 *   <li>{@code BrownedOut}: the roboRIO's own brownout flag
 *   <li>{@code Robot State}, {@code Flywheel/State}, {@code Intake/State}: for the dashboard's
 *       per-state rows, e.g. whether {@code RobotCurrentLimits} really cut drive current while
 *       SCORING
 * </ul>
 */
public final class HootLogging {
    /**
     * What the toggle starts at on a roboRIO that has never had it set. After that the persisted
     * NetworkTables value wins, so changing this constant does not override a value already set.
     */
    public static final boolean kEnabledByDefault = false;

    /** Only on the real robot, and only if a USB stick is plugged in; see the constructor. */
    private static final String kUsbLogPath = "/u/logs";

    private final Superstructure m_superstructure;

    private final BooleanEntry m_enabled;
    private final BooleanPublisher m_active;
    private boolean m_logging = false;

    // Last value written of each custom signal. null/false forces a write on the next check.
    private Enum<?> m_lastRobotState;
    private Enum<?> m_lastFlywheelState;
    private Enum<?> m_lastIntakeState;
    private boolean m_lastBrownedOut;

    public HootLogging(Superstructure superstructure) {
        m_superstructure = superstructure;

        // This class decides when logging runs; Phoenix's own auto-start would defeat the toggle.
        SignalLogger.enableAutoLogging(false);

        // The roboRIO's internal flash is small and slow to write; CTRE stops logging below 5 MB
        // free. A USB stick avoids both. Without one, Phoenix's default location is used.
        if (RobotBase.isReal() && new File("/u").isDirectory()) {
            CtreUtil.reportIfNotOk("hoot set path", SignalLogger.setPath(kUsbLogPath));
        }

        var table = NetworkTableInstance.getDefault().getTable("SmartDashboard/Hoot Logging");
        var enabledTopic = table.getBooleanTopic("Enabled");
        m_enabled = enabledTopic.getEntry(kEnabledByDefault);
        m_enabled.setDefault(kEnabledByDefault);
        enabledTopic.setPersistent(true);
        m_active = table.getBooleanTopic("Active").publish();
        m_active.set(false);
    }

    /** Call once per loop, from robotPeriodic. */
    public void periodic() {
        boolean wanted = m_enabled.get();
        if (wanted != m_logging) {
            if (wanted) {
                start();
            } else {
                stop();
            }
        }
        if (!m_logging) {
            return;
        }

        boolean brownedOut = RobotController.isBrownedOut();
        if (brownedOut != m_lastBrownedOut) {
            SignalLogger.writeBoolean("BrownedOut", brownedOut);
            m_lastBrownedOut = brownedOut;
        }

        // Enum identity compare: these are the same constants every loop, so this allocates nothing.
        Enum<?> robotState = m_superstructure.getRobotState();
        if (robotState != m_lastRobotState) {
            SignalLogger.writeString("Robot State", robotState.name());
            m_lastRobotState = robotState;
        }
        Enum<?> flywheelState = m_superstructure.m_flywheel.getState();
        if (flywheelState != m_lastFlywheelState) {
            SignalLogger.writeString("Flywheel/State", flywheelState.name());
            m_lastFlywheelState = flywheelState;
        }
        Enum<?> intakeState = m_superstructure.m_intake.getState();
        if (intakeState != m_lastIntakeState) {
            SignalLogger.writeString("Intake/State", intakeState.name());
            m_lastIntakeState = intakeState;
        }
    }

    private void start() {
        CtreUtil.reportIfNotOk("hoot logging start", SignalLogger.start());
        m_logging = true;
        m_active.set(true);
        // A new log starts empty, so every custom signal needs its current value written again.
        m_lastRobotState = null;
        m_lastFlywheelState = null;
        m_lastIntakeState = null;
        SignalLogger.writeBoolean("BrownedOut", RobotController.isBrownedOut());
        m_lastBrownedOut = RobotController.isBrownedOut();
    }

    private void stop() {
        CtreUtil.reportIfNotOk("hoot logging stop", SignalLogger.stop());
        m_logging = false;
        m_active.set(false);
    }
}
