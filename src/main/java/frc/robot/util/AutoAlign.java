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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Drives the robot to an Autopilot target.
 */
public class AutoAlign extends Command {
    /**
     * Selects how AutoAlign should generate the requested robot heading.
     */
    public enum RotationControlMode {
        /** Let the drivetrain heading PID move directly to the target angle. */
        UNPROFILED_PID,
        /**
         * Limit the heading setpoint velocity before sending it to the drivetrain PID.
         */
        VELOCITY_LIMITED_PROFILE
    }

    public static class AutoAlignConstants {
        public static double DEFAULT_MAX_VELOCITY = 5.5; // physical max is 5.5 m/s
        public static double DEFAULT_ACCELERATION = 23; // Calculated from swerve slip current
        public static double DEFAULT_JERK = 6.0;

        public static double PROFILED_ROTATION_DEFAULT_VELOCITY = Math.PI * 2; // rad/s
        public static double PROFILED_ROTATION_SLOW_VELOCITY = Math.PI * 1; // rad/s

        public static double PROFILED_ROTATION_DEFAULT_ACCELERATION = 6 * Math.PI; // rad/s^2
        public static double ROTATION_PROFILE_PERIOD = 0.020; // seconds
        public static double ROTATION_PROFILE_MAX_PERIOD = 0.060; // seconds

        // Constants are listed as (velocity, acceleration, jerk) or (acceleration,
        // jerk)
        public static APConstraints SLOW_DRIVE_CONSTRAINTS = new APConstraints(1.3, DEFAULT_ACCELERATION, 20);
        public static APConstraints SLOW_CRAWL_CONSTRAINTS = new APConstraints(0.5, DEFAULT_ACCELERATION, 20);

        public static APConstraints VELOCITY_LIMITED_CONSTRAINTS = new APConstraints(
                DEFAULT_MAX_VELOCITY,
                DEFAULT_ACCELERATION,
                DEFAULT_JERK);
        public static APConstraints HIGH_JERK_CONSTRAINTS = new APConstraints(DEFAULT_MAX_VELOCITY,
                DEFAULT_ACCELERATION, 60);
        public static APConstraints DEFAULT_CONSTRAINTS = new APConstraints(DEFAULT_ACCELERATION, DEFAULT_JERK);
        public static PrimitiveRotationProfile.Constraints DEFAULT_ROTATION_CONSTRAINTS = new PrimitiveRotationProfile.Constraints(
                PROFILED_ROTATION_DEFAULT_ACCELERATION);
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
            .withHeadingPID(5, 0, 0); // Replace with constants later

    protected SwerveDriveState swerveState = new SwerveDriveState();

    /**
     * Creates an AutoAlign command with direct drivetrain heading PID.
     *
     * @param targetPose The desired field-relative target pose.
     * @param drivetrain The drivetrain subsystem to command.
     * @param profile    The Autopilot profile used for translation and completion
     *                   tolerances.
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
     * Creates an AutoAlign command with velocity-limited profiled rotation.
     *
     * @param targetPose                  The desired field-relative target pose.
     * @param drivetrain                  The drivetrain subsystem to command.
     * @param profile                     The Autopilot profile used for translation
     *                                    and completion tolerances.
     * @param profiledRotationMaxVelocity Max profiled heading velocity, in rad/s.
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

    public AutoAlign(
            APTarget target,
            CommandSwerveDrivetrain drivetrain,
            APConstraints constraints,
            RotationControlMode rotationControlMode,
            double profiledRotationMaxVelocity) {
        this(target, drivetrain, new APProfile(constraints), rotationControlMode, profiledRotationMaxVelocity);
    }

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
     * Creates an AutoAlign command with direct drivetrain heading PID that ends
     * once
     * the robot is within a distance of the target translation.
     *
     * @param profile    The Autopilot profile used for translation and completion
     *                   tolerances.
     * @param targetPose The desired field-relative target pose.
     * @param drivetrain The drivetrain subsystem to command.
     * @param distance   The distance from the target translation that ends the
     *                   command.
     * @return AutoAlign command decorated with the distance end condition.
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
     * Creates an AutoAlign command with velocity-limited profiled rotation that
     * ends
     * once the robot is within a distance of the target translation.
     *
     * @param profile                     The Autopilot profile used for translation
     *                                    and completion tolerances.
     * @param targetPose                  The desired field-relative target pose.
     * @param drivetrain                  The drivetrain subsystem to command.
     * @param distance                    The distance from the target translation
     *                                    that ends the command.
     * @param profiledRotationMaxVelocity Max profiled heading velocity, in rad/s.
     * @return AutoAlign command decorated with the distance end condition.
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

    public AutoAlign withVelocityLimitedRotation(double profiledRotationMaxVelocity) {
        return new AutoAlign(
                m_target,
                m_drivetrain,
                m_profile,
                RotationControlMode.VELOCITY_LIMITED_PROFILE,
                m_rotationConstraints,
                profiledRotationMaxVelocity);
    }

    public APProfile getProfile() {
        return m_profile;
    }

    public RotationControlMode getRotationControlMode() {
        return m_rotationControlMode;
    }

    public double getProfiledRotationMaxVelocity() {
        return m_profiledRotationMaxVelocity;
    }

    @Override
    public void initialize() {
        swerveState = m_drivetrain.getState();
        m_rotationProfile.reset(
                swerveState.Pose.getRotation().getRadians(),
                swerveState.Speeds.omegaRadiansPerSecond,
                swerveState.Timestamp);
    }

    @Override
    public void execute() {
        swerveState = m_drivetrain.getState();
        APResult out = kAutopilot.calculate(swerveState.Pose, swerveState.Speeds, m_target);

        applyDriveRequest(out);
    }

    protected void applyDriveRequest(APResult out) {
        if (m_rotationControlMode == RotationControlMode.VELOCITY_LIMITED_PROFILE) {
            m_rotationProfile.update(out.targetAngle().getRadians(), swerveState.Timestamp);
            m_drivetrain.setControl(m_request
                    .withVelocityX(out.vx())
                    .withVelocityY(out.vy())
                    .withTargetDirection(Rotation2d.fromRadians(m_rotationProfile.positionRadians()))
                    .withTargetRateFeedforward(m_rotationProfile.velocityRadiansPerSecond())
                    .withMaxAbsRotationalRate(m_rotationProfile.maxVelocity()));
            return;
        }

        m_drivetrain.setControl(m_request
                .withVelocityX(out.vx())
                .withVelocityY(out.vy())
                .withTargetRateFeedforward(0)
                .withMaxAbsRotationalRate(0)
                .withTargetDirection(out.targetAngle()));
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setControl(m_driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return kAutopilot.atTarget(m_drivetrain.getState().Pose, m_target);
    }
}
