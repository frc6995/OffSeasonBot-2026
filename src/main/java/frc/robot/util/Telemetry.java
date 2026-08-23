package frc.robot.util;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;


public class Telemetry {
    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot pose for field positioning */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    private static final double BASE_X = Units.feetToMeters(3);
    public static final Mechanism2d MECH_VISUALIZER =
      new Mechanism2d(BASE_X * 2, Units.feetToMeters(7));
    private static final MechanismRoot2d MECH_VISUALIZER_ROOT =
      MECH_VISUALIZER.getRoot("root", BASE_X, Units.inchesToMeters(7.5));


    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    /**
     * Minimum spacing between pose publishes. telemeterize() is driven by Phoenix's odometry
     * thread, which runs at up to 250Hz - far faster than any dashboard renders - so publishing
     * every callback would put 5x the useful traffic on NetworkTables for no visible benefit.
     */
    private static final double kMinPublishIntervalSeconds = 0.02;

    private double m_lastPublishTimestamp = Double.NEGATIVE_INFINITY;

    private final double[] m_poseArray = new double[3];
    private final double[] m_moduleStatesArray = new double[8];
    private final double[] m_moduleTargetsArray = new double[8];

    public Telemetry() {
        // The Field2d type string never changes, so publish it once here rather than on every
        // telemeterize() callback (which runs on the odometry thread at up to 250Hz).
        fieldTypePub.set("Field2d");
    }

    /**
     * Accept the swerve drive state and telemeterize it to SmartDashboard and
     * SignalLogger.
     *
     * <p>Called from Phoenix's odometry thread on every state update, not from the main loop.
     */
    public void telemeterize(SwerveDriveState state) {
        double now = Timer.getFPGATimestamp();
        if (now - m_lastPublishTimestamp < kMinPublishIntervalSeconds) {
            return;
        }
        m_lastPublishTimestamp = now;

        // /* Telemeterize the swerve drive state */
        m_poseArray[0] = state.Pose.getX();
        m_poseArray[1] = state.Pose.getY();
        m_poseArray[2] = state.Pose.getRotation().getDegrees();

        // /* Telemeterize the pose to a Field2d */
        fieldPub.set(m_poseArray);
    }
}
