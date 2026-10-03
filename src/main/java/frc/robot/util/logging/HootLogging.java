package frc.robot.util.logging;

import java.io.File;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.Superstructure;
import frc.robot.util.CtreUtil;


public final class HootLogging {
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
