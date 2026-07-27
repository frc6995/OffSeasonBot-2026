package frc.robot.util;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * An auto‑alignment command that drives the robot to a target pose while holding a fixed
 * heading (or a cardinalized version of it). The translational motion is handled by the
 * underlying autopilot, and the rotation is kept constant at the specified heading.
 *
 * <p>This class extends {@link AutoAlign} and overrides the initialization, execution,
 * and completion logic to enforce a fixed heading throughout the alignment.
 */
public class AutoAlignFixedHeading extends AutoAlign {

    /** The actual autopilot target that includes the fixed heading. */
    private APTarget m_realTarget;

    /** The fixed heading to maintain during alignment (or the base heading if cardinalizing). */
    private final Rotation2d m_heading;

    /** If {@code true}, the heading is snapped to the nearest cardinal direction (0, ±90, 180). */
    private final boolean m_cardinalize;

    /**
     * Constructs a new AutoAlignFixedHeading with a given target pose, entry angle, drivetrain,
     * fixed heading, and rotation control mode. Uses the default profile constraints.
     *
     * @param targetPose           the desired final pose (translation and initial heading,
     *                             but the heading will be overridden by {@code heading})
     * @param entryAngle           the entry angle for the autopilot approach
     * @param drivetrain           the swerve drivetrain subsystem
     * @param heading              the fixed heading to hold during alignment
     * @param rotationControlMode  the mode for controlling rotation (e.g., profiled or direct)
     */
    public AutoAlignFixedHeading(
            Pose2d targetPose,
            Rotation2d entryAngle,
            CommandSwerveDrivetrain drivetrain,
            Rotation2d heading,
            RotationControlMode rotationControlMode) {
        super(
                targetPose,
                entryAngle,
                drivetrain,
                kDefaultProfile,
                rotationControlMode,
                AutoAlignConstants.PROFILED_ROTATION_DEFAULT_VELOCITY);
        m_heading = heading;
        m_cardinalize = false;
    }

    /**
     * Constructs a new AutoAlignFixedHeading with a target pose, drivetrain, fixed heading,
     * and rotation control mode. Uses default constraints and no entry angle.
     *
     * @param targetPose           the desired final pose (heading will be overridden)
     * @param drivetrain           the swerve drivetrain subsystem
     * @param heading              the fixed heading to hold during alignment
     * @param rotationControlMode  the mode for controlling rotation
     */
    public AutoAlignFixedHeading(
            Pose2d targetPose,
            CommandSwerveDrivetrain drivetrain,
            Rotation2d heading,
            RotationControlMode rotationControlMode) {
        this(new APTarget(targetPose), drivetrain, heading, AutoAlignConstants.DEFAULT_CONSTRAINTS, rotationControlMode);
    }

    /**
     * Constructs a new AutoAlignFixedHeading with a target pose, entry angle, drivetrain,
     * a cardinalize flag, and rotation control mode. Uses default constraints and the
     * current robot heading as the base heading (which will be cardinalized if
     * {@code cardinalize} is true).
     *
     * @param targetPose           the desired final pose (heading will be overridden)
     * @param entryAngle           the entry angle for the autopilot approach
     * @param drivetrain           the swerve drivetrain subsystem
     * @param cardinalize          if {@code true}, the heading is snapped to a cardinal direction
     * @param rotationControlMode  the mode for controlling rotation
     */
    public AutoAlignFixedHeading(
            Pose2d targetPose,
            Rotation2d entryAngle,
            CommandSwerveDrivetrain drivetrain,
            boolean cardinalize,
            RotationControlMode rotationControlMode) {
        this(
                new APTarget(targetPose).withEntryAngle(entryAngle),
                drivetrain,
                AutoAlignConstants.DEFAULT_CONSTRAINTS,
                cardinalize,
                rotationControlMode);
    }

    /**
     * Constructs a new AutoAlignFixedHeading with a target pose, drivetrain, a cardinalize flag,
     * and rotation control mode. Uses default constraints and the current robot heading as the
     * base heading (which will be cardinalized if {@code cardinalize} is true).
     *
     * @param targetPose           the desired final pose (heading will be overridden)
     * @param drivetrain           the swerve drivetrain subsystem
     * @param cardinalize          if {@code true}, the heading is snapped to a cardinal direction
     * @param rotationControlMode  the mode for controlling rotation
     */
    public AutoAlignFixedHeading(
            Pose2d targetPose,
            CommandSwerveDrivetrain drivetrain,
            boolean cardinalize,
            RotationControlMode rotationControlMode) {
        this(new APTarget(targetPose), drivetrain, AutoAlignConstants.DEFAULT_CONSTRAINTS, cardinalize, rotationControlMode);
    }

    /**
     * Constructs a new AutoAlignFixedHeading with a full {@link APTarget}, drivetrain,
     * fixed heading, constraints, and rotation control mode. No cardinalization is applied.
     *
     * @param target               the autopilot target (its heading will be overridden)
     * @param drivetrain           the swerve drivetrain subsystem
     * @param fixedHeading         the fixed heading to hold during alignment
     * @param constraints          the motion constraints for the autopilot
     * @param rotationControlMode  the mode for controlling rotation
     */
    public AutoAlignFixedHeading(
            APTarget target,
            CommandSwerveDrivetrain drivetrain,
            Rotation2d fixedHeading,
            APConstraints constraints,
            RotationControlMode rotationControlMode) {
        super(target, drivetrain, constraints, rotationControlMode, AutoAlignConstants.PROFILED_ROTATION_DEFAULT_VELOCITY);
        m_heading = fixedHeading;
        m_cardinalize = false;
    }

    /**
     * Constructs a new AutoAlignFixedHeading with a full {@link APTarget}, drivetrain,
     * constraints, a cardinalize flag, and rotation control mode. The base heading is taken
     * from the current robot pose and may be cardinalized depending on the flag.
     *
     * @param target               the autopilot target (its heading will be overridden)
     * @param drivetrain           the swerve drivetrain subsystem
     * @param constraints          the motion constraints for the autopilot
     * @param cardinalize          if {@code true}, the heading is snapped to a cardinal direction
     * @param rotationControlMode  the mode for controlling rotation
     */
    public AutoAlignFixedHeading(
            APTarget target,
            CommandSwerveDrivetrain drivetrain,
            APConstraints constraints,
            boolean cardinalize,
            RotationControlMode rotationControlMode) {
        super(target, drivetrain, constraints, rotationControlMode, AutoAlignConstants.PROFILED_ROTATION_DEFAULT_VELOCITY);
        m_cardinalize = cardinalize;
        m_heading = drivetrain.state().Pose.getRotation();
    }

    /**
     * Snaps a heading to the nearest cardinal direction (0°, ±90°, 180°) using a quadrant‑based
     * partitioning. The input is first wrapped to the range [-180°, 180°).
     *
     * @param heading  the original heading
     * @return         the cardinalized heading (one of 0°, 90°, -90°, 180°, or -180°)
     */
    public static Rotation2d cardinalizeHeading(Rotation2d heading) {
        double hdegrees = MathUtil.inputModulus(heading.getDegrees(), -180, 180);
        if (hdegrees >= -135 && hdegrees < -45) {
            return Rotation2d.fromDegrees(-90);
        }
        else if (hdegrees >= -45 && hdegrees < 45) {
            return Rotation2d.kZero;
        }
        else if (hdegrees >= 45 && hdegrees < 135) {
            return Rotation2d.fromDegrees(90);
        }
        else if (hdegrees >= 135 && hdegrees < 180) {
            return Rotation2d.k180deg;
        }
        else if (hdegrees >= -180 && hdegrees < -135) {
            return Rotation2d.fromDegrees(-180);
        }
        else {
            return Rotation2d.kZero;
        }
    }

    /**
     * Snaps a heading to either 0° or 180° (north/south cardinalization) based on whether
     * the heading is closer to 0° or 180°. The input is first wrapped to [-180°, 180°).
     *
     * @param heading  the original heading
     * @return         {@link Rotation2d#kZero} if the heading is within ±90° of 0°,
     *                 otherwise {@link Rotation2d#k180deg} (or -180°).
     */
    public static Rotation2d cardinalizeHeadingNS(Rotation2d heading) {
        double hdegrees = MathUtil.inputModulus(heading.getDegrees(), -180, 180);
        if (hdegrees >= -90 && hdegrees <= 90) {
            return Rotation2d.kZero;
        }
        else if (hdegrees > 90 && hdegrees <= 180) {
            return Rotation2d.k180deg;
        }
        else if (hdegrees < -90 && hdegrees >= -180) {
            return Rotation2d.fromDegrees(-180);
        }
        else {
            return Rotation2d.kZero;
        }
    }

    /**
     * Initializes the command by building the actual autopilot target with the fixed
     * (or cardinalized) heading, then calls the superclass initialization.
     */
    @Override
    public void initialize() {
        super.initialize();

        Rotation2d targetHeading = m_cardinalize ? cardinalizeHeading(m_heading) : m_heading;
        Pose2d targetPose = new Pose2d(m_target.getReference().getTranslation(), targetHeading);

        m_realTarget = m_target.withReference(targetPose);
    }

    /**
     * Executes the command by calculating the autopilot output for the fixed‑heading target
     * and applying the resulting drive request.
     */
    @Override
    public void execute() {
        swerveState = m_drivetrain.getState();
        APResult out = kAutopilot.calculate(swerveState.Pose, swerveState.Speeds, m_realTarget);

        applyDriveRequest(out);
    }

    @Override
    public boolean isFinished() {
        return kAutopilot.atTarget(m_drivetrain.getState().Pose, m_realTarget);
    }
}