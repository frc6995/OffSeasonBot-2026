package frc.robot.util;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import java.util.function.UnaryOperator;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * A command that drives the robot to a specified field-relative target pose using
 * an Autopilot motion profile for translation. The robot can either use the
 * drivetrain's built-in heading PID directly, or apply a velocity‑limited
 * profiled rotation to smooth heading changes, and can optionally hold a fixed
 * (or cardinalized) heading instead of the target pose's own rotation.
 *
 * <p>Instances are immutable configuration objects - every {@code with*} method returns a
 * <b>new</b> {@code AutoAlign} rather than mutating the receiver, so a base command can be built
 * once and specialized per use site with a fluent chain, e.g.:
 *
 * <pre>{@code
 * AutoAlign.toPose(targetPose, drivetrain)
 *         .withProfile(AutoAlign.slowDriveProfile())
 *         .withEntryAngle(Rotation2d.fromDegrees(180))
 *         .withProfiledRotation(AutoAlign.RotationProfile.DEFAULT)
 *         .untilWithinTolerance(Centimeters.of(15));
 * }</pre>
 *
 * <p>This command is designed to be used with a Phoenix 6 swerve drivetrain and
 * the "autopilot" library for translational motion planning.
 *
 * @see Autopilot
 * @see APProfile
 * @see APTarget
 */
public class AutoAlign extends Command {

    /**
     * Defines how the commanded robot heading is generated.
     */
    public enum RotationControlMode {
        /**
         * Uses the drivetrain's internal heading PID controller directly.
         * No profile limiting is applied to the heading setpoint; the PID is given
         * the target angle and expected to track it.
         */
        UNPROFILED_PID,

        /**
         * Limits the heading setpoint velocity by passing the target angle through
         * a motion profile. The resulting profiled position and velocity are sent
         * to the drivetrain's heading PID as a feedforward, reducing overshoot and
         * smoothing rotation.
         */
        VELOCITY_LIMITED_PROFILE
    }

    /**
     * Holds configurable constants used by the AutoAlign command and its profiles.
     * These values can be tuned for different robot behaviours.
     */
    public static class AutoAlignConstants {
        public static final double DEFAULT_MAX_VELOCITY = 5.5;
        public static final double DEFAULT_ACCELERATION = 23;
        public static final double DEFAULT_JERK = 6.0;

        public static final double PROFILED_ROTATION_DEFAULT_VELOCITY = Math.PI;
        public static final double PROFILED_ROTATION_SLOW_VELOCITY = Math.PI * 0.3;

        public static final double PROFILED_ROTATION_DEFAULT_ACCELERATION = 6 * Math.PI;
        /** Update period of the rotation profile (seconds). */
        public static final double ROTATION_PROFILE_PERIOD = 0.020;
        /** Maximum allowable period between loop updates (seconds). */
        public static final double ROTATION_PROFILE_MAX_PERIOD = 0.060;

        /*
         * Constraints and profiles are handed out by factory methods rather than held as static
         * fields. APConstraints and APProfile are both mutable - their with* methods assign to
         * their own fields and return this, they do not copy - and Autopilot keeps the profile
         * reference for the lifetime of the command. A shared static would therefore let one
         * caller's withErrorXY(...) silently retune every command on the robot, and marking the
         * field final would not prevent it: final stops reassignment, not mutation. A fresh
         * instance per call makes that impossible instead of merely discouraged.
         */

        public static APConstraints slowDriveConstraints() {
            return new APConstraints(1.6, DEFAULT_ACCELERATION, 60);
        }

        public static APConstraints slowCrawlConstraints() {
            return new APConstraints(0.5, DEFAULT_ACCELERATION, 20);
        }

        public static APConstraints velocityLimitedConstraints() {
            return new APConstraints(DEFAULT_MAX_VELOCITY, DEFAULT_ACCELERATION, DEFAULT_JERK);
        }

        public static APConstraints highJerkConstraints() {
            return new APConstraints(DEFAULT_MAX_VELOCITY, DEFAULT_ACCELERATION, 60);
        }

        public static APConstraints defaultConstraints() {
            return new APConstraints(DEFAULT_MAX_VELOCITY, DEFAULT_ACCELERATION, DEFAULT_JERK);
        }

        /**
         * Tolerances used to build a full {@link APProfile} out of a bare {@link APConstraints}.
         * Matches the tolerances used by {@link AutoAlign#slowDriveProfile()} and friends - an
         * {@link APProfile} built with only constraints and no tolerances defaults both error
         * axes to zero, which {@link com.therekrab.autopilot.Autopilot#atTarget} can never
         * actually satisfy.
         *
         * <p>Safe as constants: {@link Distance} and {@link edu.wpi.first.units.measure.Angle}
         * are immutable.
         */
        public static final Distance DEFAULT_ERROR_XY = Centimeters.of(8);
        public static final edu.wpi.first.units.measure.Angle DEFAULT_ERROR_THETA = Degrees.of(2.5);
        public static final Distance DEFAULT_BEELINE_RADIUS = Centimeters.of(8);

        /**
         * Default rotation profile constraints (acceleration only, velocity is limited separately).
         * Safe as a constant: {@link PrimitiveRotationProfile.Constraints} is immutable.
         */
        public static final PrimitiveRotationProfile.Constraints DEFAULT_ROTATION_CONSTRAINTS =
                new PrimitiveRotationProfile.Constraints(PROFILED_ROTATION_DEFAULT_ACCELERATION);
    }

    /**
     * Bundles the constraints and max velocity for a velocity-limited profiled rotation, so
     * {@link #withProfiledRotation(RotationProfile)} can configure both in one call. Both fields
     * are immutable, so instances are safe to share and reuse across commands.
     *
     * @param constraints                 Rotation profile acceleration constraints.
     * @param maxVelocityRadiansPerSecond Maximum profiled heading velocity, in rad/s.
     */
    public record RotationProfile(PrimitiveRotationProfile.Constraints constraints, double maxVelocityRadiansPerSecond) {
        /** A rotation profile with the given max acceleration and velocity (both rad/s^2, rad/s). */
        public static RotationProfile of(double maxAccelerationRadiansPerSecondSquared, double maxVelocityRadiansPerSecond) {
            return new RotationProfile(
                    new PrimitiveRotationProfile.Constraints(maxAccelerationRadiansPerSecondSquared),
                    maxVelocityRadiansPerSecond);
        }

        /** The standard profiled-rotation velocity/acceleration, used when no tuning is needed. */
        public static final RotationProfile DEFAULT = new RotationProfile(
                AutoAlignConstants.DEFAULT_ROTATION_CONSTRAINTS,
                AutoAlignConstants.PROFILED_ROTATION_DEFAULT_VELOCITY);

        /** A slower profiled-rotation velocity, for approaches that should turn gently. */
        public static final RotationProfile SLOW = new RotationProfile(
                AutoAlignConstants.DEFAULT_ROTATION_CONSTRAINTS,
                AutoAlignConstants.PROFILED_ROTATION_SLOW_VELOCITY);
    }

    /** A fresh tight-tolerance profile with no velocity cap beyond the default constraints. */
    public static APProfile defaultProfile() {
        return new APProfile(AutoAlignConstants.defaultConstraints())
                .withErrorXY(Centimeters.of(6))
                .withErrorTheta(Degrees.of(1.5))
                .withBeelineRadius(Centimeters.of(8));
    }

    /** A fresh velocity-limited profile with loose translational tolerance. */
    public static APProfile defaultVelocityLimitedProfile() {
        return new APProfile(AutoAlignConstants.velocityLimitedConstraints())
                .withErrorXY(Centimeters.of(12))
                .withErrorTheta(Degrees.of(1.5))
                .withBeelineRadius(Centimeters.of(8));
    }

    /** A fresh slow-approach profile, for the last leg of an alignment. */
    public static APProfile slowDriveProfile() {
        return new APProfile(AutoAlignConstants.slowDriveConstraints())
                .withErrorXY(Centimeters.of(8))
                .withErrorTheta(Degrees.of(2.5))
                .withBeelineRadius(Centimeters.of(8));
    }

    /** A fresh crawl profile, slower still than {@link #slowDriveProfile()}. */
    public static APProfile slowCrawlProfile() {
        return new APProfile(AutoAlignConstants.slowCrawlConstraints())
                .withErrorXY(Centimeters.of(8))
                .withErrorTheta(Degrees.of(2.5))
                .withBeelineRadius(Centimeters.of(8));
    }

    /** A fresh high-jerk profile, for fast drive-through waypoints. */
    public static APProfile highJerkProfile() {
        return new APProfile(AutoAlignConstants.highJerkConstraints())
                .withErrorXY(Centimeters.of(8))
                .withErrorTheta(Degrees.of(2.5))
                .withBeelineRadius(Centimeters.of(8));
    }

    /**
     * Snaps a heading to the nearest cardinal direction (0°, ±90°, 180°) using a quadrant‑based
     * partitioning. The input is first wrapped to the range [-180°, 180°).
     *
     * @param heading the original heading
     * @return the cardinalized heading (one of 0°, 90°, -90°, 180°, or -180°)
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
     * @param heading the original heading
     * @return {@link Rotation2d#kZero} if the heading is within ±90° of 0°,
     *         otherwise {@link Rotation2d#k180deg} (or -180°).
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

    // ============= Immutable configuration =============
    protected final APTarget m_target;
    protected final CommandSwerveDrivetrain m_drivetrain;
    protected final APProfile m_profile;
    protected final RotationControlMode m_rotationControlMode;
    protected final RotationProfile m_rotationProfileConfig;
    /**
     * When non-null, overrides the target's heading with a fixed value computed once at
     * {@link #initialize()} time from the robot's heading at that moment (see
     * {@link #withFixedHeading}, {@link #withCardinalizedHeading}). When null, the target's own
     * reference rotation is used, as passed to Autopilot normally.
     */
    protected final UnaryOperator<Rotation2d> m_headingOverride;

    // ============= Runtime-only state =============
    protected final Autopilot kAutopilot;
    protected final PrimitiveRotationProfile m_rotationProfileRuntime;

    protected final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric();
    protected final SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withHeadingPID(5, 0, 0);

    /** Cached swerve drive state for use during execution. */
    protected SwerveDriveState swerveState = new SwerveDriveState();

    /** The target actually sent to Autopilot each cycle; computed once in {@link #initialize()}. */
    protected APTarget m_effectiveTarget;

    /**
     * Constructs an AutoAlign command targeting a pose, with direct (unprofiled) heading PID and
     * the default profile. Use the {@code with*} methods to further configure it, e.g.
     * {@link #withProfile}, {@link #withEntryAngle}, {@link #withProfiledRotation}.
     *
     * @param targetPose The desired field-relative target pose (translation + rotation).
     * @param drivetrain The drivetrain subsystem to command.
     */
    public AutoAlign(Pose2d targetPose, CommandSwerveDrivetrain drivetrain) {
        this(new APTarget(targetPose), drivetrain, defaultProfile(),
                RotationControlMode.UNPROFILED_PID, RotationProfile.DEFAULT, null);
    }

    /**
     * Constructs an AutoAlign command targeting a pose with a specific translation profile.
     *
     * @param targetPose The desired field-relative target pose.
     * @param drivetrain The drivetrain subsystem to command.
     * @param profile    The Autopilot profile used for translation and completion tolerances.
     */
    public AutoAlign(Pose2d targetPose, CommandSwerveDrivetrain drivetrain, APProfile profile) {
        this(new APTarget(targetPose), drivetrain, profile,
                RotationControlMode.UNPROFILED_PID, RotationProfile.DEFAULT, null);
    }

    /**
     * Fully parameterized constructor backing every {@code with*} method - each of those simply
     * calls this with one field changed, so it stays the single place that assembles the runtime
     * Autopilot/rotation-profile objects from the immutable configuration.
     */
    private AutoAlign(
            APTarget target,
            CommandSwerveDrivetrain drivetrain,
            APProfile profile,
            RotationControlMode rotationControlMode,
            RotationProfile rotationProfileConfig,
            UnaryOperator<Rotation2d> headingOverride) {
        m_target = target;
        m_drivetrain = drivetrain;
        m_profile = profile;
        m_rotationControlMode = rotationControlMode;
        m_rotationProfileConfig = rotationProfileConfig;
        m_headingOverride = headingOverride;

        m_rotationProfileRuntime = new PrimitiveRotationProfile(
                rotationProfileConfig.constraints(),
                rotationProfileConfig.maxVelocityRadiansPerSecond(),
                AutoAlignConstants.ROTATION_PROFILE_PERIOD,
                AutoAlignConstants.ROTATION_PROFILE_MAX_PERIOD);

        kAutopilot = new Autopilot(profile);

        addRequirements(drivetrain);
    }

    /**
     * Entry point for the fluent builder style, e.g.
     * {@code AutoAlign.toPose(pose, drivetrain).withProfile(...).untilWithinTolerance(...)}.
     * Equivalent to {@code new AutoAlign(targetPose, drivetrain)}.
     *
     * @param targetPose The desired field-relative target pose.
     * @param drivetrain The drivetrain subsystem to command.
     * @return A new AutoAlign with default profile, unprofiled heading PID, and no heading override.
     */
    public static AutoAlign toPose(Pose2d targetPose, CommandSwerveDrivetrain drivetrain) {
        return new AutoAlign(targetPose, drivetrain);
    }

    /**
     * Returns a new AutoAlign with the given translation profile (constraints + tolerances),
     * preserving every other setting.
     *
     * @param profile The Autopilot profile used for translation and completion tolerances.
     */
    public AutoAlign withProfile(APProfile profile) {
        return new AutoAlign(m_target, m_drivetrain, profile, m_rotationControlMode, m_rotationProfileConfig, m_headingOverride);
    }

    /**
     * Returns a new AutoAlign with the given entry angle (the direction Autopilot approaches the
     * target translation from), preserving every other setting.
     *
     * @param entryAngle The desired entry angle at the target.
     */
    public AutoAlign withEntryAngle(Rotation2d entryAngle) {
        return new AutoAlign(
                m_target.withEntryAngle(entryAngle),
                m_drivetrain, m_profile, m_rotationControlMode, m_rotationProfileConfig, m_headingOverride);
    }

    /**
     * Returns a new AutoAlign that drives its heading through a velocity-limited motion profile
     * (see {@link RotationControlMode#VELOCITY_LIMITED_PROFILE}) using the given profile,
     * preserving every other setting.
     *
     * @param rotationProfile The rotation profile constraints and max velocity to use.
     */
    public AutoAlign withProfiledRotation(RotationProfile rotationProfile) {
        return new AutoAlign(
                m_target, m_drivetrain, m_profile, RotationControlMode.VELOCITY_LIMITED_PROFILE, rotationProfile, m_headingOverride);
    }

    /**
     * Returns a new AutoAlign that drives its heading directly with the drivetrain's PID (see
     * {@link RotationControlMode#UNPROFILED_PID}), preserving every other setting.
     */
    public AutoAlign withUnprofiledRotation() {
        return new AutoAlign(
                m_target, m_drivetrain, m_profile, RotationControlMode.UNPROFILED_PID, m_rotationProfileConfig, m_headingOverride);
    }

    /**
     * Returns a new AutoAlign that holds a fixed heading throughout the move, instead of the
     * target pose's own rotation. Preserves every other setting.
     *
     * @param heading The heading to hold for the duration of the command.
     */
    public AutoAlign withFixedHeading(Rotation2d heading) {
        return new AutoAlign(
                m_target, m_drivetrain, m_profile, m_rotationControlMode, m_rotationProfileConfig, currentHeading -> heading);
    }

    /**
     * Returns a new AutoAlign that holds the robot's heading at {@link #initialize()} time,
     * snapped to the nearest cardinal direction (see {@link #cardinalizeHeading}), instead of the
     * target pose's own rotation. Preserves every other setting.
     */
    public AutoAlign withCardinalizedHeading() {
        return new AutoAlign(
                m_target, m_drivetrain, m_profile, m_rotationControlMode, m_rotationProfileConfig,
                AutoAlign::cardinalizeHeading);
    }

    /**
     * Returns a new AutoAlign that holds the robot's heading at {@link #initialize()} time,
     * snapped to north or south (see {@link #cardinalizeHeadingNS}), instead of the target pose's
     * own rotation. Preserves every other setting.
     */
    public AutoAlign withCardinalizedHeadingNS() {
        return new AutoAlign(
                m_target, m_drivetrain, m_profile, m_rotationControlMode, m_rotationProfileConfig,
                AutoAlign::cardinalizeHeadingNS);
    }

    /**
     * Returns a command that runs this alignment until the robot's translation is within
     * {@code tolerance} of the target - independent of, and usually looser than, the Autopilot
     * profile's own completion tolerance (which also still applies, since the two conditions are
     * simply raced by {@link Command#until}).
     *
     * <p>This is the same distance-based end condition every profile in this class uses
     * internally; no custom {@code isFinished()} override is added here or needed by callers.
     *
     * @param tolerance The distance from the target translation at which the command ends.
     * @return This command, decorated with the distance-based end condition.
     */
    public Command untilWithinTolerance(Distance tolerance) {
        return until(TriggerUtil.isWithinRadius(
                () -> m_target.getReference().getTranslation(),
                () -> m_drivetrain.state().Pose,
                () -> tolerance));
    }

    /**
     * Returns the current Autopilot profile used for translation.
     *
     * @return The APProfile.
     */
    public APProfile getProfile() {
        return m_profile;
    }

    /**
     * Returns the current rotation control mode.
     *
     * @return The RotationControlMode.
     */
    public RotationControlMode getRotationControlMode() {
        return m_rotationControlMode;
    }

    /**
     * Returns the current rotation profile (constraints and max velocity), used when the
     * rotation control mode is {@link RotationControlMode#VELOCITY_LIMITED_PROFILE}.
     *
     * @return The RotationProfile.
     */
    public RotationProfile getRotationProfile() {
        return m_rotationProfileConfig;
    }

    @Override
    public void initialize() {
        // getStateCopy(), not state(): the three fields read below have to describe the same
        // odometry tick. state() hands back Phoenix's live internal state object, which the
        // odometry thread rewrites at up to 250Hz, so Pose/Speeds/Timestamp read off it can come
        // from different ticks - and a Timestamp that disagrees with the pose feeds a wrong dt
        // straight into the rotation profile. getStateCopy() clones under Phoenix's state lock.
        swerveState = m_drivetrain.getStateCopy();
        // Reset the rotation profile to the current heading and angular velocity.
        m_rotationProfileRuntime.reset(
                swerveState.Pose.getRotation().getRadians(),
                swerveState.Speeds.omegaRadiansPerSecond,
                swerveState.Timestamp);

        // Fixed/cardinalized heading is resolved once, from the heading at the start of the
        // move - not recomputed every cycle - so the target heading stays constant for the
        // duration of the command.
        m_effectiveTarget = m_headingOverride == null
                ? m_target
                : m_target.withReference(new Pose2d(
                        m_target.getReference().getTranslation(),
                        m_headingOverride.apply(swerveState.Pose.getRotation())));
    }

    @Override
    public void execute() {
        // Coherent snapshot required - see initialize(). applyDriveRequest() below also reads
        // swerveState.Timestamp, so Pose, Speeds and Timestamp must all be from one tick.
        swerveState = m_drivetrain.getStateCopy();
        // Compute translational setpoints from Autopilot.
        APResult out = kAutopilot.calculate(swerveState.Pose, swerveState.Speeds, m_effectiveTarget);

        applyDriveRequest(out);
    }

    /**
     * Applies the drive request to the drivetrain, handling both rotation control modes.
     *
     * @param out The APResult containing translational velocities and target angle.
     */
    protected void applyDriveRequest(APResult out) {
        if (m_rotationControlMode == RotationControlMode.VELOCITY_LIMITED_PROFILE) {
            // Update the rotation profile with the target angle and current timestamp.
            m_rotationProfileRuntime.update(out.targetAngle().getRadians(), swerveState.Timestamp);
            // Send a field-centric request with profiled heading and feedforward.
            m_drivetrain.setControl(m_request
                    .withVelocityX(out.vx())
                    .withVelocityY(out.vy())
                    .withTargetDirection(Rotation2d.fromRadians(m_rotationProfileRuntime.positionRadians()))
                    .withTargetRateFeedforward(m_rotationProfileRuntime.velocityRadiansPerSecond())
                    .withMaxAbsRotationalRate(m_rotationProfileRuntime.maxVelocity()));
            return;
        }

        // UNPROFILED_PID: direct heading control with no feedforward.
        m_drivetrain.setControl(m_request
                .withVelocityX(out.vx())
                .withVelocityY(out.vy())
                .withTargetRateFeedforward(0)
                .withMaxAbsRotationalRate(0)
                .withTargetDirection(out.targetAngle()));
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot.
        m_drivetrain.setControl(m_driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        // Check if the Autopilot considers the robot within tolerance of the (possibly
        // heading-overridden) target.
        return kAutopilot.atTarget(m_drivetrain.state().Pose, m_effectiveTarget);
    }
}
