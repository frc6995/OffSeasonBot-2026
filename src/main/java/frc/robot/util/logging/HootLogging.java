package frc.robot.util.logging;

import java.io.File;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.CtreUtil;
import frc.robot.subsystems.vision.ATVision;

public final class HootLogging {
    public static final boolean kEnabled = true;

    /** Only on the real robot, and only if a USB stick is plugged in; see the constructor. */
    private static final String kUsbLogPath = "/u/logs";

    private final Superstructure m_superstructure;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final ATVision mVision;
    private Pose2d m_lastRobotPose;

    // Last value written of each custom signal. null forces a write on the first loop.
    private Enum<?> m_lastRobotState;
    private Enum<?> m_lastFlywheelState;
    private Enum<?> m_lastIntakeState;
    private boolean m_lastBrownedOut;
    private Enum<?> m_lastHoodState;
    private Enum<?> m_lastTurretState;
    private Enum<?> m_lastDyeRotorSpinState;
    private Enum<?> m_lastDyeRotorIndexState;
    private Pose2d m_lastVisionEstimate; // null => write on first loop an estimate exists

    public HootLogging(Superstructure superstructure, CommandSwerveDrivetrain drivetrain, ATVision vision) {
        m_superstructure = superstructure;
        m_drivetrain = drivetrain;
        mVision = vision;

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

        // Raw vision estimate (pre-fusion). equals() compare: each accepted estimate is a fresh
        // Pose2d, so this writes only when a new estimate was accepted.
        Pose2d visionEstimate = mVision.getLastAcceptedEstimate();
        if (visionEstimate != m_lastVisionEstimate) {
            if (visionEstimate != null) {
                SignalLogger.writeDouble("Vision/EstimateX", visionEstimate.getX());
                SignalLogger.writeDouble("Vision/EstimateY", visionEstimate.getY());
                SignalLogger.writeDouble("Vision/EstimateTheta", visionEstimate.getRotation().getRadians());
            }
            m_lastVisionEstimate = visionEstimate;
        }

        // Fused robot pose (odometry + vision, post-Kalman). Written only when it changes.
        Pose2d robotPose = m_drivetrain.getPose();
        if (!robotPose.equals(m_lastRobotPose)) {
            SignalLogger.writeDouble("Robot/PoseX", robotPose.getX());
            SignalLogger.writeDouble("Robot/PoseY", robotPose.getY());
            SignalLogger.writeDouble("Robot/PoseTheta", robotPose.getRotation().getRadians());
            m_lastRobotPose = robotPose;
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

        Enum<?> hoodState = m_superstructure.m_hood.getState();
        if (hoodState != m_lastHoodState) {
            SignalLogger.writeString("Hood/State", hoodState.name());
            m_lastHoodState = hoodState;
        }
        Enum<?> turretState = m_superstructure.m_turret.getState();
        if (turretState != m_lastTurretState) {
            SignalLogger.writeString("Turret/State", turretState.name());
            m_lastTurretState = turretState;
        }
        Enum<?> dyeRotorSpinState = m_superstructure.m_dyeRotor.getSpinState();
        if (dyeRotorSpinState != m_lastDyeRotorSpinState) {
            SignalLogger.writeString("DyeRotor/SpinState", dyeRotorSpinState.name());
            m_lastDyeRotorSpinState = dyeRotorSpinState;
        }
        Enum<?> dyeRotorIndexState = m_superstructure.m_dyeRotor.getIndexState();
        if (dyeRotorIndexState != m_lastDyeRotorIndexState) {
            SignalLogger.writeString("DyeRotor/IndexState", dyeRotorIndexState.name());
            m_lastDyeRotorIndexState = dyeRotorIndexState;
        }

    }
}
