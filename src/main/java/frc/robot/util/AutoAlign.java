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
     * Holds configurable constants used by the AutoAlign command and its profiles.
     * These values can be tuned for different robot behaviours.
     */
    public static class AutoAlignConstants {
        public static double DEFAULT_MAX_VELOCITY = 5.5;
        public static double DEFAULT_ACCELERATION = 23;
        public static double DEFAULT_JERK = 6.0;

        public static double PROFILED_ROTATION_DEFAULT_VELOCITY = Math.PI;
        public static double PROFILED_ROTATION_SLOW_VELOCITY = Math.PI * 0.3;

        public static double PROFILED_ROTATION_DEFAULT_ACCELERATION = 6 * Math.PI;
        /** Update period of the rotation profile (seconds). */
        public static double ROTATION_PROFILE_PERIOD = 0.020;
        /** Maximum allowable period between loop updates (seconds). */
        public static double ROTATION_PROFILE_MAX_PERIOD = 0.060;

        public static APConstraints SLOW_DRIVE_CONSTRAINTS = new APConstraints(1.6, DEFAULT_ACCELERATION, 60);
        public static APConstraints SLOW_CRAWL_CONSTRAINTS = new APConstraints(0.5, DEFAULT_ACCELERATION, 20);
        public static APConstraints VELOCITY_LIMITED_CONSTRAINTS = new APConstraints(DEFAULT_MAX_VELOCITY, DEFAULT_ACCELERATION, DEFAULT_JERK);
        public static APConstraints HIGH_JERK_CONSTRAINTS = new APConstraints(DEFAULT_MAX_VELOCITY, DEFAULT_ACCELERATION, 60);
        public static APConstraints DEFAULT_CONSTRAINTS =
                new APConstraints(DEFAULT_MAX_VELOCITY, DEFAULT_ACCELERATION, DEFAULT_JERK);

        /**
         * Tolerances used to build a full {@link APProfile} out of a bare {@link APConstraints},
         * for constructors that only take constraints (see {@link AutoAlign#AutoAlign(APTarget,
         * CommandSwerveDrivetrain, APConstraints, RotationControlMode, double)}). Matches the
         * tolerances used by {@link AutoAlign#kSlowDriveProfile} and friends - an
         * {@link APProfile} built with only constraints and no tolerances defaults both error
         * axes to zero, which {@link com.therekrab.autopilot.Autopilot#atTarget} can never
         * actually satisfy.
         */
        public static Distance DEFAULT_ERROR_XY = Centimeters.of(8);
        public static Angle DEFAULT_ERROR_THETA = Degrees.of(2.5);
        public static Distance DEFAULT_BEELINE_RADIUS = Centimeters.of(8);

        /** Default rotation profile constraints (acceleration only, velocity is limited separately). */
        public static PrimitiveRotationProfile.Constraints DEFAULT_ROTATION_CONSTRAINTS =
                new PrimitiveRotationProfile.Constraints(PROFILED_ROTATION_DEFAULT_ACCELERATION);
    }

    public static APProfile kDefaultProfile = new APProfile(AutoAlignConstants.DEFAULT_CONSTRAINTS)
            .withErrorXY(Centimeters.of(6))
            .withErrorTheta(Degrees.of(1.5))
            .withBeelineRadius(Centimeters.of(8));

    public static APProfile kDefaultVelocityLimitedProfile = new APProfile(
            AutoAlignConstants.VELOCITY_LIMITED_CONSTRAINTS)
            .withErrorXY(Centimeters.of(12))
            .withErrorTheta(Degrees.of(1.5))
            .withBeelineRadius(Centimeters.of(8));

    public static APProfile kSlowDriveProfile = new APProfile(
            AutoAlignConstants.SLOW_DRIVE_CONSTRAINTS)
            .withErrorXY(Centimeters.of(8))
            .withErrorTheta(Degrees.of(2.5))
            .withBeelineRadius(Centimeters.of(8));

    public static APProfile kSlowCrawlProfile = new APProfile(
            AutoAlignConstants.SLOW_CRAWL_CONSTRAINTS)
            .withErrorXY(Centimeters.of(8))
            .withErrorTheta(Degrees.of(2.5))
            .withBeelineRadius(Centimeters.of(8));

                public static APProfile kHighJerkProfile = new APProfile(
            AutoAlignConstants.HIGH_JERK_CONSTRAINTS)
            .withErrorXY(Centimeters.of(8))
            .withErrorTheta(Degrees.of(2.5))
            .withBeelineRadius(Centimeters.of(8));


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

    /**
     * Creates a new AutoAlign command with a modified translation profile.
     *
     * @param profileModifier A function that takes the current profile and returns a new one.
     * @return A new AutoAlign instance with the modified profile, preserving all other settings.
     */
    public AutoAlign withModifiedProfile(java.util.function.Function<APProfile, APProfile> profileModifier) {
        APProfile modifiedProfile = profileModifier.apply(m_profile);
        return new AutoAlign(
                m_target,
                m_drivetrain,
                modifiedProfile,
                m_rotationControlMode,
                m_rotationConstraints,
                m_profiledRotationMaxVelocity);
    }

    /**
     * Creates a new AutoAlign command that uses velocity‑limited profiled rotation
     * with the specified maximum velocity.
     *
     * @param profiledRotationMaxVelocity New maximum profiled heading velocity (rad/s).
     * @return A new AutoAlign instance with the rotation mode forced to VELOCITY_LIMITED_PROFILE.
     */
    public AutoAlign withVelocityLimitedRotation(double profiledRotationMaxVelocity) {
        return new AutoAlign(
                m_target,
                m_drivetrain,
                m_profile,
                RotationControlMode.VELOCITY_LIMITED_PROFILE,
                m_rotationConstraints,
                profiledRotationMaxVelocity);
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

    @Override
    public void initialize() {
        swerveState = m_drivetrain.getState();
        // Reset the rotation profile to the current heading and angular velocity.
        m_rotationProfile.reset(
                swerveState.Pose.getRotation().getRadians(),
                swerveState.Speeds.omegaRadiansPerSecond,
                swerveState.Timestamp);
    }

    @Override
    public void execute() {
        swerveState = m_drivetrain.getState();
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
        return kAutopilot.atTarget(m_drivetrain.getState().Pose, m_target);
    }
}