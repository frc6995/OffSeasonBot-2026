package frc.robot.util;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.Function;

/**
 * A command that drives the robot to a specified field-relative target pose using
 * an Autopilot motion profile for translation. The robot can either use the
 * drivetrain's built-in heading PID directly, or apply a velocity‑limited
 * profiled rotation to smooth heading changes.
 *
 * <p>This command is designed to be used with a Phoenix 6 swerve drivetrain and
 * the "autopilot" library for translational motion planning. It supports
 * configurable constraints for both translation and rotation, and can be
 * composed with other commands (e.g., to end early based on distance).
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
     * Immutable configuration for the velocity-limited heading profile used by
     * {@link RotationControlMode#VELOCITY_LIMITED_PROFILE}.
     *
     * <p>This is deliberately a plain config object rather than the stateful
     * {@link PrimitiveRotationProfile}: a running profile holds position/velocity/timestamp state and
     * must never be shared between commands. {@link AutoAlign} builds a fresh
     * {@link PrimitiveRotationProfile} from this config each time a command is created, so a single
     * {@code RotationProfile} is safe to reuse across every auto and binding on the robot.
     */
    public static final class RotationProfile {
        private final PrimitiveRotationProfile.Constraints m_constraints;
        private final double m_maxVelocity;

        /**
         * Builds a rotation profile with explicit constraints and maximum velocity.
         *
         * @param constraints rotation profile constraints (acceleration).
         * @param maxVelocity maximum heading velocity, in rad/s.
         */
        public RotationProfile(PrimitiveRotationProfile.Constraints constraints, double maxVelocity) {
            m_constraints = constraints;
            m_maxVelocity = maxVelocity;
        }

        /**
         * Builds a rotation profile using the default rotation constraints.
         *
         * @param maxVelocity maximum heading velocity, in rad/s.
         */
        public RotationProfile(double maxVelocity) {
            this(AutoAlignConstants.DEFAULT_ROTATION_CONSTRAINTS, maxVelocity);
        }

        /**
         * Factory for a rotation profile using the default rotation constraints.
         *
         * @param maxVelocity maximum heading velocity, in rad/s.
         * @return a new {@link RotationProfile}.
         */
        public static RotationProfile of(double maxVelocity) {
            return new RotationProfile(maxVelocity);
        }

        /**
         * Factory for a rotation profile with explicit constraints and maximum velocity.
         *
         * @param constraints rotation profile constraints (acceleration).
         * @param maxVelocity maximum heading velocity, in rad/s.
         * @return a new {@link RotationProfile}.
         */
        public static RotationProfile of(
                PrimitiveRotationProfile.Constraints constraints, double maxVelocity) {
            return new RotationProfile(constraints, maxVelocity);
        }

        /**
         * Returns the rotation profile constraints.
         *
         * @return the acceleration-limited constraints.
         */
        public PrimitiveRotationProfile.Constraints constraints() {
            return m_constraints;
        }

        /**
         * Returns the maximum heading velocity.
         *
         * @return the velocity in rad/s.
         */
        public double maxVelocity() {
            return m_maxVelocity;
        }

        /**
         * Builds a fresh stateful rotation profile from this configuration. Each command must own its
         * own instance; never cache or share the result.
         *
         * @return a new {@link PrimitiveRotationProfile}.
         */
        public PrimitiveRotationProfile build() {
            return new PrimitiveRotationProfile(
                    m_constraints,
                    m_maxVelocity,
                    AutoAlignConstants.ROTATION_PROFILE_PERIOD,
                    AutoAlignConstants.ROTATION_PROFILE_MAX_PERIOD);
        }
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
         * Tolerances used to build a full {@link APProfile} out of a bare {@link APConstraints},
         * for constructors that only take constraints (see {@link AutoAlign#AutoAlign(APTarget,
         * CommandSwerveDrivetrain, APConstraints, RotationControlMode, double)}). Matches the
         * tolerances used by {@link AutoAlign#slowDriveProfile()} and friends - an
         * {@link APProfile} built with only constraints and no tolerances defaults both error
         * axes to zero, which {@link com.therekrab.autopilot.Autopilot#atTarget} can never
         * actually satisfy.
         *
         * <p>Safe as constants: {@link Distance} and {@link Angle} are immutable.
         */
        public static final Distance DEFAULT_ERROR_XY = Centimeters.of(8);
        public static final Angle DEFAULT_ERROR_THETA = Degrees.of(2.5);
        public static final Distance DEFAULT_BEELINE_RADIUS = Centimeters.of(8);

        /**
         * Default rotation profile constraints (acceleration only, velocity is limited separately).
         * Safe as a constant: {@link PrimitiveRotationProfile.Constraints} is immutable.
         */
        public static final PrimitiveRotationProfile.Constraints DEFAULT_ROTATION_CONSTRAINTS =
                new PrimitiveRotationProfile.Constraints(PROFILED_ROTATION_DEFAULT_ACCELERATION);
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

    protected final Autopilot kAutopilot;
    protected final APTarget m_target;
    protected final CommandSwerveDrivetrain m_drivetrain;
    protected final APProfile m_profile;
    protected final RotationControlMode m_rotationControlMode;
    protected final PrimitiveRotationProfile.Constraints m_rotationConstraints;
    protected final double m_profiledRotationMaxVelocity;
    protected final PrimitiveRotationProfile m_rotationProfile;

    protected final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric();
    protected final SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withHeadingPID(5, 0, 0);

    /** Cached swerve drive state for use during execution. */
    protected SwerveDriveState swerveState = new SwerveDriveState();

    /**
     * Constructs an AutoAlign command with the default translation profile and direct (unprofiled)
     * drivetrain heading PID. This is the minimal entry point for auto routines, which can then
     * refine the command through the fluent {@code with*}/{@code until*} methods.
     *
     * @param targetPose The desired field-relative target pose (translation + rotation).
     * @param drivetrain The drivetrain subsystem to command.
     */
    public AutoAlign(Pose2d targetPose, CommandSwerveDrivetrain drivetrain) {
        this(targetPose, drivetrain, defaultProfile());
    }

    /**
     * Constructs an AutoAlign command with direct (unprofiled) drivetrain heading PID.
     *
     * @param targetPose The desired field-relative target pose (translation + rotation).
     * @param drivetrain The drivetrain subsystem to command.
     * @param profile    The Autopilot profile used for translation and completion tolerances.
     */
    public AutoAlign(
            Pose2d targetPose,
            CommandSwerveDrivetrain drivetrain,
            APProfile profile) {
        this(
                new APTarget(targetPose),
                drivetrain,
                profile,
                RotationControlMode.UNPROFILED_PID,
                AutoAlignConstants.DEFAULT_ROTATION_CONSTRAINTS,
                AutoAlignConstants.PROFILED_ROTATION_DEFAULT_VELOCITY);
    }

    /**
     * Constructs an AutoAlign command with velocity‑limited profiled rotation.
     *
     * @param targetPose                  The desired field-relative target pose.
     * @param drivetrain                  The drivetrain subsystem to command.
     * @param profile                     The Autopilot profile used for translation and completion tolerances.
     * @param profiledRotationMaxVelocity Maximum profiled heading velocity, in rad/s.
     */
    public AutoAlign(
            Pose2d targetPose,
            CommandSwerveDrivetrain drivetrain,
            APProfile profile,
            double profiledRotationMaxVelocity) {
        this(
                new APTarget(targetPose),
                drivetrain,
                profile,
                RotationControlMode.VELOCITY_LIMITED_PROFILE,
                AutoAlignConstants.DEFAULT_ROTATION_CONSTRAINTS,
                profiledRotationMaxVelocity);
    }

    /**
     * Constructs an AutoAlign command with an explicit entry angle (the final
     * orientation at the target may be approached from a specific direction).
     *
     * @param targetPose                  The desired field-relative target pose.
     * @param entryAngle                  The desired entry angle at the target (used by the Autopilot).
     * @param drivetrain                  The drivetrain subsystem to command.
     * @param profile                     The Autopilot profile for translation and tolerances.
     * @param rotationControlMode         The rotation control mode to use.
     * @param profiledRotationMaxVelocity Maximum profiled heading velocity (rad/s), if applicable.
     */
    public AutoAlign(
            Pose2d targetPose,
            Rotation2d entryAngle,
            CommandSwerveDrivetrain drivetrain,
            APProfile profile,
            RotationControlMode rotationControlMode,
            double profiledRotationMaxVelocity) {
        this(
                new APTarget(targetPose).withEntryAngle(entryAngle),
                drivetrain,
                profile,
                rotationControlMode,
                AutoAlignConstants.DEFAULT_ROTATION_CONSTRAINTS,
                profiledRotationMaxVelocity);
    }

    /**
     * Constructs an AutoAlign command with an APTarget, custom constraints, and a
     * rotation control mode.
     *
     * @param target                      The APTarget (may include entry angle).
     * @param drivetrain                  The drivetrain subsystem to command.
     * @param constraints                 Translational constraints for the Autopilot profile.
     * @param rotationControlMode         The rotation control mode.
     * @param profiledRotationMaxVelocity Maximum profiled heading velocity (rad/s), if applicable.
     */
    public AutoAlign(
            APTarget target,
            CommandSwerveDrivetrain drivetrain,
            APConstraints constraints,
            RotationControlMode rotationControlMode,
            double profiledRotationMaxVelocity) {
        // A bare `new APProfile(constraints)` defaults both error tolerances to zero, which
        // Autopilot#atTarget can never actually satisfy (see AutoAlignConstants.DEFAULT_ERROR_XY
        // javadoc) - so give it the same real tolerances every named profile in this class uses.
        this(
                target,
                drivetrain,
                new APProfile(constraints)
                        .withErrorXY(AutoAlignConstants.DEFAULT_ERROR_XY)
                        .withErrorTheta(AutoAlignConstants.DEFAULT_ERROR_THETA)
                        .withBeelineRadius(AutoAlignConstants.DEFAULT_BEELINE_RADIUS),
                rotationControlMode,
                profiledRotationMaxVelocity);
    }

    /**
     * Constructs an AutoAlign command with an APTarget and a full APProfile.
     *
     * @param target                      The APTarget (may include entry angle).
     * @param drivetrain                  The drivetrain subsystem to command.
     * @param profile                     The Autopilot profile for translation and tolerances.
     * @param rotationControlMode         The rotation control mode.
     * @param profiledRotationMaxVelocity Maximum profiled heading velocity (rad/s), if applicable.
     */
    public AutoAlign(
            APTarget target,
            CommandSwerveDrivetrain drivetrain,
            APProfile profile,
            RotationControlMode rotationControlMode,
            double profiledRotationMaxVelocity) {
        this(
                target,
                drivetrain,
                profile,
                rotationControlMode,
                AutoAlignConstants.DEFAULT_ROTATION_CONSTRAINTS,
                profiledRotationMaxVelocity);
    }

    /**
     * Fully parameterized constructor.
     *
     * @param target                      The APTarget (may include entry angle).
     * @param drivetrain                  The drivetrain subsystem to command.
     * @param profile                     The Autopilot profile for translation and tolerances.
     * @param rotationControlMode         The rotation control mode.
     * @param rotationConstraints         Rotation profile constraints (acceleration).
     * @param profiledRotationMaxVelocity Maximum profiled heading velocity (rad/s), if applicable.
     */
    public AutoAlign(
            APTarget target,
            CommandSwerveDrivetrain drivetrain,
            APProfile profile,
            RotationControlMode rotationControlMode,
            PrimitiveRotationProfile.Constraints rotationConstraints,
            double profiledRotationMaxVelocity) {
        m_target = target;
        m_drivetrain = drivetrain;
        m_profile = profile;
        m_rotationControlMode = rotationControlMode;
        m_rotationConstraints = rotationConstraints;
        m_profiledRotationMaxVelocity = profiledRotationMaxVelocity;
        m_rotationProfile = new PrimitiveRotationProfile(
                rotationConstraints,
                profiledRotationMaxVelocity,
                AutoAlignConstants.ROTATION_PROFILE_PERIOD,
                AutoAlignConstants.ROTATION_PROFILE_MAX_PERIOD);

        kAutopilot = new Autopilot(profile);

        addRequirements(drivetrain);
    }

    /**
     * Returns an independent copy of this command with the given configuration applied.
     *
     * <p>Every fluent method delegates here so that no setting can be dropped by a partial copy, and
     * so that adding a new field only requires touching these overloads. The copy shares the
     * drivetrain but owns its own {@link PrimitiveRotationProfile}, so a config chain can never leak
     * rotation state back into the original command.
     *
     * @param target              the target to command toward.
     * @param profile             the translation profile to use.
     * @param rotationControlMode the heading control strategy.
     * @param rotationConstraints constraints for the profiled heading controller.
     * @param maxVelocity         maximum profiled heading velocity, in rad/s.
     * @return a new, fully-configured {@link AutoAlign}.
     */
    private AutoAlign copy(
            APTarget target,
            APProfile profile,
            RotationControlMode rotationControlMode,
            PrimitiveRotationProfile.Constraints rotationConstraints,
            double maxVelocity) {
        return new AutoAlign(
                target, m_drivetrain, profile, rotationControlMode, rotationConstraints, maxVelocity);
    }

    /**
     * Returns a copy of this command with only the target changed.
     *
     * @param target the new target.
     * @return a new {@link AutoAlign} preserving every other setting.
     */
    private AutoAlign copy(APTarget target) {
        return copy(
                target,
                m_profile,
                m_rotationControlMode,
                m_rotationConstraints,
                m_profiledRotationMaxVelocity);
    }

    /**
     * Returns a copy of this command with only the translation profile changed.
     *
     * @param profile the new translation profile.
     * @return a new {@link AutoAlign} preserving every other setting.
     */
    private AutoAlign copy(APProfile profile) {
        return copy(
                m_target,
                profile,
                m_rotationControlMode,
                m_rotationConstraints,
                m_profiledRotationMaxVelocity);
    }

    /**
     * Returns a copy of this command with only the rotation control mode changed.
     *
     * @param rotationControlMode the new rotation control mode.
     * @return a new {@link AutoAlign} preserving every other setting.
     */
    private AutoAlign copy(RotationControlMode rotationControlMode) {
        return copy(
                m_target,
                m_profile,
                rotationControlMode,
                m_rotationConstraints,
                m_profiledRotationMaxVelocity);
    }

    /**
     * Creates an AutoAlign command with direct (unprofiled) heading PID that ends
     * once the robot's translation is within a specified distance of the target.
     *
     * @param profile    The Autopilot profile used for translation and completion tolerances.
     * @param targetPose The desired field-relative target pose.
     * @param drivetrain The drivetrain subsystem to command.
     * @param distance   The distance from the target translation at which the command ends.
     * @return An AutoAlign command decorated with a distance-based end condition.
     */
    public static Command toPoseUntilWithinDistance(
            APProfile profile,
            Pose2d targetPose,
            CommandSwerveDrivetrain drivetrain,
            Distance distance) {
        return withDistanceCancel(
                new AutoAlign(targetPose, drivetrain, profile),
                targetPose,
                drivetrain,
                distance);
    }

    /**
     * Creates an AutoAlign command with velocity‑limited profiled rotation that ends
     * once the robot's translation is within a specified distance of the target.
     *
     * @param profile                     The Autopilot profile used for translation and tolerances.
     * @param targetPose                  The desired field-relative target pose.
     * @param drivetrain                  The drivetrain subsystem to command.
     * @param distance                    The distance from the target translation at which the command ends.
     * @param profiledRotationMaxVelocity Maximum profiled heading velocity (rad/s).
     * @return An AutoAlign command decorated with a distance-based end condition.
     */
    public static Command toPoseUntilWithinDistance(
            APProfile profile,
            Pose2d targetPose,
            CommandSwerveDrivetrain drivetrain,
            Distance distance,
            double profiledRotationMaxVelocity) {
        return withDistanceCancel(
                new AutoAlign(targetPose, drivetrain, profile, profiledRotationMaxVelocity),
                targetPose,
                drivetrain,
                distance);
    }

    /**
     * Creates an AutoAlign command that ends when the robot is within a distance
     * of the target translation, using an explicit entry angle and rotation control mode.
     *
     * @param profile                     The Autopilot profile for translation and tolerances.
     * @param targetPose                  The desired field-relative target pose.
     * @param entryAngle                  The desired entry angle (used by the Autopilot).
     * @param drivetrain                  The drivetrain subsystem to command.
     * @param distance                    The distance from the target translation at which the command ends.
     * @param rotationControlMode         The rotation control mode.
     * @param profiledRotationMaxVelocity Maximum profiled heading velocity (rad/s), if applicable.
     * @return An AutoAlign command that ends when within distance.
     */
    public static Command toPoseUntilWithinDistance(
            APProfile profile,
            Pose2d targetPose,
            Rotation2d entryAngle,
            CommandSwerveDrivetrain drivetrain,
            Distance distance,
            RotationControlMode rotationControlMode,
            double profiledRotationMaxVelocity) {
        return new AutoAlign(
                targetPose,
                entryAngle,
                drivetrain,
                profile,
                rotationControlMode,
                profiledRotationMaxVelocity)
                .until(TriggerUtil.isWithinRadius(
                        () -> targetPose.getTranslation(),
                        () -> drivetrain.state().Pose,
                        () -> distance));
    }

    /**
     * Helper method that decorates an AutoAlign command with a distance-based end condition.
     *
     * @param autoAlign  The AutoAlign command to decorate.
     * @param targetPose The target pose used for distance checking.
     * @param drivetrain The drivetrain subsystem (to get current pose).
     * @param distance   The distance threshold.
     * @return The decorated command.
     */
    private static Command withDistanceCancel(
            AutoAlign autoAlign,
            Pose2d targetPose,
            CommandSwerveDrivetrain drivetrain,
            Distance distance) {
        return autoAlign.until(TriggerUtil.isWithinRadius(
                () -> targetPose.getTranslation(),
                () -> drivetrain.state().Pose,
                () -> distance));
    }

    // ===================== FLUENT CONFIGURATION API =====================
    //
    // Every method below returns a NEW AutoAlign built through copy(...), leaving the receiver
    // untouched. That makes a configured command safe to hand to multiple autos and means a chain
    // can be reused as a "base" configuration without one auto's tweaks leaking into another's.

    /**
     * Returns a copy of this command targeting a different {@link APTarget}. The target carries the
     * reference pose, optional entry angle, end velocity, and rotation radius, so it is the most
     * general way to retarget an alignment.
     *
     * @param target the new target.
     * @return a new command aimed at {@code target}.
     */
    public AutoAlign withTarget(APTarget target) {
        return copy(target);
    }

    /**
     * Returns a copy of this command targeting a different pose. Any entry angle already set on the
     * target is preserved; use {@link #withoutEntryAngle()} to clear it.
     *
     * @param targetPose the new field-relative target pose.
     * @return a new command aimed at {@code targetPose}.
     */
    public AutoAlign withTarget(Pose2d targetPose) {
        return withTarget(m_target.withReference(targetPose));
    }

    /**
     * Returns a copy of this command with the given entry angle, so the robot approaches the target
     * travelling in that direction.
     *
     * @param entryAngle the desired travel direction at the target.
     * @return a new command with the entry angle applied.
     */
    public AutoAlign withEntryAngle(Rotation2d entryAngle) {
        return withTarget(m_target.withEntryAngle(entryAngle));
    }

    /**
     * Returns a copy of this command with no entry angle, so the robot drives straight at the target.
     *
     * @return a new command with the entry angle cleared.
     */
    public AutoAlign withoutEntryAngle() {
        return withTarget(m_target.withoutEntryAngle());
    }

    /**
     * Returns a copy of this command with an end velocity, for drive-through waypoints where the
     * robot should not stop on arrival.
     *
     * @param velocity the desired end velocity, in m/s.
     * @return a new command with the end velocity applied.
     */
    public AutoAlign withTargetVelocity(double velocity) {
        return withTarget(m_target.withVelocity(velocity));
    }

    /**
     * Returns a copy of this command that only respects the target heading once within the given
     * radius of the target.
     *
     * @param radius the distance from the target within which the heading goal is respected.
     * @return a new command with the rotation radius applied.
     */
    public AutoAlign withRotationRadius(Distance radius) {
        return withTarget(m_target.withRotationRadius(radius));
    }

    /**
     * Returns a copy of this command using a different translation profile.
     *
     * @param profile the Autopilot profile to use for translation and completion tolerances.
     * @return a new command using {@code profile}.
     */
    public AutoAlign withProfile(APProfile profile) {
        return copy(profile);
    }

    /**
     * Returns a copy of this command with the translation profile modified by the given function.
     * The function receives the current profile, so this is the escape hatch for tuning a named
     * profile without rebuilding it from scratch.
     *
     * @param profileModifier a function that takes the current profile and returns a new one.
     * @return a new command with the modified profile, preserving every other setting.
     */
    public AutoAlign withModifiedProfile(Function<APProfile, APProfile> profileModifier) {
        return withProfile(profileModifier.apply(m_profile));
    }

    /**
     * Returns a copy of this command using a different heading control strategy.
     *
     * @param rotationControlMode the rotation control mode to use.
     * @return a new command using {@code rotationControlMode}.
     */
    public AutoAlign withRotationControlMode(RotationControlMode rotationControlMode) {
        return copy(rotationControlMode);
    }

    /**
     * Returns a copy of this command that limits the heading setpoint through a rotation profile.
     *
     * @param rotationProfile the rotation profile configuration to use.
     * @return a new command with {@link RotationControlMode#VELOCITY_LIMITED_PROFILE} enabled.
     */
    public AutoAlign withProfiledRotation(RotationProfile rotationProfile) {
        return copy(
                m_target,
                m_profile,
                RotationControlMode.VELOCITY_LIMITED_PROFILE,
                rotationProfile.constraints(),
                rotationProfile.maxVelocity());
    }

    /**
     * Returns a copy of this command that limits the heading setpoint through a rotation profile with
     * the default rotation constraints.
     *
     * @param maxVelocity maximum profiled heading velocity, in rad/s.
     * @return a new command with profiled rotation enabled.
     */
    public AutoAlign withProfiledRotation(double maxVelocity) {
        return withProfiledRotation(RotationProfile.of(maxVelocity));
    }

    /**
     * Returns a copy of this command that limits the heading setpoint through a rotation profile with
     * explicit constraints.
     *
     * @param constraints rotation profile constraints (acceleration).
     * @param maxVelocity maximum profiled heading velocity, in rad/s.
     * @return a new command with profiled rotation enabled.
     */
    public AutoAlign withProfiledRotation(
            PrimitiveRotationProfile.Constraints constraints, double maxVelocity) {
        return withProfiledRotation(RotationProfile.of(constraints, maxVelocity));
    }

    /**
     * Alias for {@link #withProfiledRotation(double)}.
     *
     * @param profiledRotationMaxVelocity maximum profiled heading velocity, in rad/s.
     * @return a new command with profiled rotation enabled.
     */
    public AutoAlign withVelocityLimitedRotation(double profiledRotationMaxVelocity) {
        return withProfiledRotation(profiledRotationMaxVelocity);
    }

    /**
     * Returns a copy of this command that drives the heading with the drivetrain's heading PID
     * directly, with no velocity limiting.
     *
     * @return a new command using {@link RotationControlMode#UNPROFILED_PID}.
     */
    public AutoAlign withUnprofiledRotation() {
        return withRotationControlMode(RotationControlMode.UNPROFILED_PID);
    }

    /**
     * Ends the alignment with a distance-to-target condition instead of the profile's own
     * {@code atTarget} tolerances.
     *
     * <p>This is exactly the {@code toPoseUntilWithinDistance} pattern applied to this
     * already-configured command: it returns a wrapper that runs this alignment and stops as soon as
     * the robot's translation is within {@code xy} of the target. Because a larger tolerance is
     * satisfied sooner, {@code xy} can be as large as the auto needs, and {@link #isFinished()} is
     * no longer the thing that decides whether a leg is done - the distance condition is. That is
     * what makes a large tolerance safe here: it fires before the profile's completion tolerance
     * ever can.
     *
     * <p>Returned as a {@link Command} (a decorated, terminal command) rather than an
     * {@link AutoAlign}, so call this last in a configuration chain.
     *
     * @param xy the translation tolerance.
     * @return a command that aligns to the target and ends within {@code xy} of it.
     */
    public Command untilWithinTolerance(Distance xy) {
        return withDistanceCancel(this, m_target.getReference(), m_drivetrain, xy);
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
     * Returns the maximum profiled rotational velocity.
     *
     * @return The velocity in rad/s.
     */
    public double getProfiledRotationMaxVelocity() {
        return m_profiledRotationMaxVelocity;
    }

    /**
     * Returns the current Autopilot target, including any entry angle, end velocity, or rotation
     * radius that has been configured.
     *
     * @return The APTarget.
     */
    public APTarget getTarget() {
        return m_target;
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
        m_rotationProfile.reset(
                swerveState.Pose.getRotation().getRadians(),
                swerveState.Speeds.omegaRadiansPerSecond,
                swerveState.Timestamp);
    }

    @Override
    public void execute() {
        // Coherent snapshot required - see initialize(). applyDriveRequest() below also reads
        // swerveState.Timestamp, so Pose, Speeds and Timestamp must all be from one tick.
        swerveState = m_drivetrain.getStateCopy();
        // Compute translational setpoints from Autopilot.
        APResult out = kAutopilot.calculate(swerveState.Pose, swerveState.Speeds, m_target);

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
            m_rotationProfile.update(out.targetAngle().getRadians(), swerveState.Timestamp);
            // Send a field-centric request with profiled heading and feedforward.
            m_drivetrain.setControl(m_request
                    .withVelocityX(out.vx())
                    .withVelocityY(out.vy())
                    .withTargetDirection(Rotation2d.fromRadians(m_rotationProfile.positionRadians()))
                    .withTargetRateFeedforward(m_rotationProfile.velocityRadiansPerSecond())
                    .withMaxAbsRotationalRate(m_rotationProfile.maxVelocity()));
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
        // Check if the Autopilot considers the robot within tolerance of the target.
        return kAutopilot.atTarget(m_drivetrain.state().Pose, m_target);
    }
}