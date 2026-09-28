package frc.robot.util.logging;

import java.io.File;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.Superstructure;
import frc.robot.util.CtreUtil;

/**
 * Starts CTRE's hoot signal logging when {@link #kEnabled} is set, and adds the handful of values the
 * power dashboard needs that no CTRE device reports.
 *
 * <p>This is the whole on-robot side of power logging. Phoenix records every motor's supply current
 * and supply voltage into the hoot file as the frames arrive on the bus, in its own thread, so the
 * robot loop never reads or writes a current value (see {@link CtreUtil#setPowerSignalFrequency}).
 * What this class adds per loop is a few cheap reads and comparisons; a hoot write happens only when
 * one of those values actually changes, a few dozen times a match.
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
     * Whether hoot logging runs. Logging starts at robot program start and runs until the program
     * exits, one folder of .hoot files per start. Set false for the baseline run of a loop-timing
     * A/B test, then redeploy.
     */
    public static final boolean kEnabled = true;

    /** Only on the real robot, and only if a USB stick is plugged in; see the constructor. */
    private static final String kUsbLogPath = "/u/logs";

    private final Superstructure m_superstructure;

    // Last value written of each custom signal. null forces a write on the first loop.
    private Enum<?> m_lastRobotState;
    private Enum<?> m_lastFlywheelState;
    private Enum<?> m_lastIntakeState;
    private boolean m_lastBrownedOut;

    public HootLogging(Superstructure superstructure) {
        m_superstructure = superstructure;

        // This class decides whether logging runs; Phoenix's own auto-start would ignore kEnabled.
        SignalLogger.enableAutoLogging(false);
        if (!kEnabled) {
            return;
        }

        // The roboRIO's internal flash is small and slow to write; CTRE stops logging below 5 MB
        // free. A USB stick avoids both. Without one, Phoenix's default location is used.
        if (RobotBase.isReal() && new File("/u").isDirectory()) {
            CtreUtil.reportIfNotOk("hoot set path", SignalLogger.setPath(kUsbLogPath));
        }
        CtreUtil.reportIfNotOk("hoot logging start", SignalLogger.start());

        m_lastBrownedOut = RobotController.isBrownedOut();
        SignalLogger.writeBoolean("BrownedOut", m_lastBrownedOut);
    }

    /** Call once per loop, from robotPeriodic. */
    public void periodic() {
        if (!kEnabled) {
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
}
