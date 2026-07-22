package frc.robot.util;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import com.therekrab.autopilot.Autopilot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlign extends Command {
    // Static factory methods with profile parameters

    public enum RotationControlMode {
        UNPROFILED_PID,
        VELOCITY_LIMITED_PROFILE
    }

    public static class RotationConstraints {
        public final double maxVelocity;
        public final double maxAcceleration;

        public RotationConstraints(double maxVelocity, double maxAcceleration) {
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
        }
    }

    public static class AutoAlignConstants {
        public static double DEFAULT_MAX_VELOCITY = 5.5; // physical max is 5.5 m/s^2
        public static double DEFAULT_ACCELERATION = 23; // Calculated from swerve slip current
        public static double DEFAULT_JERK = 6.0;
        public static double DEFAULT_ROTATION_MAX_VELOCITY = Math.PI *2; // rad/s
        public static double DEFAULT_ROTATION_MAX_ACCELERATION = 6 * Math.PI; // rad/s^2
        public static double ROTATION_PROFILE_PERIOD = 0.020; // seconds
        public static double ROTATION_PROFILE_MAX_PERIOD = 0.060; // seconds

        // Constants are listed as (velocity, acceleration, jerk) or (acceleration,
        // jerk)
        public static APConstraints SLOW_DRIVE_CONSTRAINTS = new APConstraints(1.3, DEFAULT_ACCELERATION, 20);
        public static APConstraints SLOW_CRAWL_CONSTRAINTS = new APConstraints(0.5, DEFAULT_ACCELERATION, 20);

        public static APConstraints VELOCITY_LIMITED_CONSTRAINTS = new APConstraints(DEFAULT_MAX_VELOCITY, DEFAULT_ACCELERATION, DEFAULT_JERK);
        public static APConstraints HIGH_JERK_CONSTRAINTS = new APConstraints(DEFAULT_MAX_VELOCITY, DEFAULT_ACCELERATION, 60);
        public static APConstraints DEFAULT_CONSTRAINTS = new APConstraints(DEFAULT_ACCELERATION, DEFAULT_JERK);
        public static RotationConstraints DEFAULT_ROTATION_CONSTRAINTS = new RotationConstraints(
                DEFAULT_ROTATION_MAX_VELOCITY,
                DEFAULT_ROTATION_MAX_ACCELERATION);
    }

    // Make profiles public so they can be accessed and modified
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
    protected final APProfile m_profile; // Store the profile being used
    protected final RotationControlMode m_rotationControlMode;
    protected final RotationConstraints m_rotationConstraints;
    protected final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric();
    protected final SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withHeadingPID(5, 0, 0); // Replace with constants later

    protected SwerveDriveState swerveState = new SwerveDriveState();
    protected double m_rotationSetpointRadians = 0;
    protected double m_rotationSetpointVelocity = 0;
    private double m_lastRotationProfileTimestamp;

    /**
     * Uses default constraints, beeline path
     * 
     * @param targetPose Pose2d to align to
     * @param drivetrain Drivetrain subsystem
     */
    public AutoAlign(Pose2d targetPose, CommandSwerveDrivetrain drivetrain) {
        this(new APTarget(targetPose), drivetrain, kDefaultProfile);
    }

    /**
     * Uses default constraints, beeline path with custom profile
     * 
     * @param target Pose2d to align to
     * @param drivetrain Drivetrain subsystem
     * @param constraints    APProfile to use for this alignment
     */
    public AutoAlign(APTarget target, CommandSwerveDrivetrain drivetrain, APConstraints constraints) {
        this(target, drivetrain, new APProfile(constraints));
    }

    /**
     * Uses default constraints, path respects entry angle
     * 
     * @param targetPose Pose2d to align to
     * @param entryAngle Entry angle to modify approach
     * @param drivetrain Drivetrain subsystem
     */
    public AutoAlign(Pose2d targetPose, Rotation2d entryAngle, CommandSwerveDrivetrain drivetrain) {
        this(targetPose, entryAngle, drivetrain, kDefaultProfile);
    }

    /**
     * Uses custom profile, path respects entry angle
     * 
     * @param targetPose Pose2d to align to
     * @param entryAngle Entry angle to modify approach
     * @param drivetrain Drivetrain subsystem
     * @param profile    APProfile to use for this alignment
     */
    public AutoAlign(Pose2d targetPose, Rotation2d entryAngle, CommandSwerveDrivetrain drivetrain, APProfile profile) {
        this(new APTarget(targetPose).withEntryAngle(entryAngle), drivetrain, profile);
    }

    public AutoAlign(
            Pose2d targetPose,
            Rotation2d entryAngle,
            CommandSwerveDrivetrain drivetrain,
            APProfile profile,
            RotationControlMode rotationControlMode) {
        this(new APTarget(targetPose).withEntryAngle(entryAngle), drivetrain, profile, rotationControlMode);
    }

    public AutoAlign(Pose2d targetPose, CommandSwerveDrivetrain drivetrain, APProfile profile) {
        this(new APTarget(targetPose), drivetrain, profile);
    }

    public AutoAlign(
            Pose2d targetPose,
            CommandSwerveDrivetrain drivetrain,
            APProfile profile,
            RotationControlMode rotationControlMode) {
        this(new APTarget(targetPose), drivetrain, profile, rotationControlMode);
    }

    /**
     * Creates an AutoAlign command that cancels once the robot is within the given
     * radius of the target pose.
     *
     * @param profile    APProfile to use for this alignment
     * @param targetPose Pose2d to align to
     * @param drivetrain Drivetrain subsystem
     * @param distance   Distance from the target pose where the command should cancel
     * @return AutoAlign command with a radius-based cancel condition
     */
    public static Command toPoseUntilWithinDistance(
            APProfile profile,
            Pose2d targetPose,
            CommandSwerveDrivetrain drivetrain,
            Distance distance) {
        return new AutoAlign(targetPose, drivetrain, profile)
                .until(TriggerUtil.isWithinRadius(
                        () -> targetPose.getTranslation(),
                        () -> drivetrain.state().Pose,
                        () -> distance));
    }

    public static Command toPoseUntilWithinDistance(
            APProfile profile,
            Pose2d targetPose,
            CommandSwerveDrivetrain drivetrain,
            Distance distance,
            RotationControlMode rotationControlMode) {
        return new AutoAlign(targetPose, drivetrain, profile, rotationControlMode)
                .until(TriggerUtil.isWithinRadius(
                        () -> targetPose.getTranslation(),
                        () -> drivetrain.state().Pose,
                        () -> distance));
    }

    /**
     * Creates an AutoAlign command that respects an entry angle and cancels once
     * the robot is within the given radius of the target pose.
     *
     * @param profile    APProfile to use for this alignment
     * @param targetPose Pose2d to align to
     * @param entryAngle Entry angle to modify approach
     * @param drivetrain Drivetrain subsystem
     * @param distance   Distance from the target pose where the command should cancel
     * @return AutoAlign command with an entry angle and radius-based cancel condition
     */
    public static Command toPoseUntilWithinDistance(
            APProfile profile,
            Pose2d targetPose,
            Rotation2d entryAngle,
            CommandSwerveDrivetrain drivetrain,
            Distance distance) {
        return new AutoAlign(targetPose, entryAngle, drivetrain, profile)
                .until(TriggerUtil.isWithinRadius(
                        () -> targetPose.getTranslation(),
                        () -> drivetrain.state().Pose,
                        () -> distance));
    }

    public static Command toPoseUntilWithinDistance(
            APProfile profile,
            Pose2d targetPose,
            Rotation2d entryAngle,
            CommandSwerveDrivetrain drivetrain,
            Distance distance,
            RotationControlMode rotationControlMode) {
        return new AutoAlign(targetPose, entryAngle, drivetrain, profile, rotationControlMode)
                .until(TriggerUtil.isWithinRadius(
                        () -> targetPose.getTranslation(),
                        () -> drivetrain.state().Pose,
                        () -> distance));
    }

    /**
     * Auto align constructor with full parameters
     * 
     * @param target     APTarget to align to
     * @param drivetrain Drivetrain subsystem
     * @param profile    APProfile to use for this alignment
     */
    public AutoAlign(APTarget target, CommandSwerveDrivetrain drivetrain, APProfile profile) {
        this(target, drivetrain, profile, RotationControlMode.UNPROFILED_PID);
    }

    public AutoAlign(
            APTarget target,
            CommandSwerveDrivetrain drivetrain,
            APProfile profile,
            RotationControlMode rotationControlMode) {
        this(target, drivetrain, profile, rotationControlMode, AutoAlignConstants.DEFAULT_ROTATION_CONSTRAINTS);
    }

    public AutoAlign(
            APTarget target,
            CommandSwerveDrivetrain drivetrain,
            APProfile profile,
            RotationControlMode rotationControlMode,
            RotationConstraints rotationConstraints) {
        this.m_target = target;
        this.m_drivetrain = drivetrain;
        this.m_profile = profile;
        this.m_rotationControlMode = rotationControlMode;
        this.m_rotationConstraints = rotationConstraints;

        kAutopilot = new Autopilot(profile);

        addRequirements(drivetrain);
    }

    /**
     * Creates a new AutoAlign with a modified version of the current profile
     * Useful for runtime adjustments
     * 
     * @param profileModifier Function to modify the profile
     */
    public AutoAlign withModifiedProfile(java.util.function.Function<APProfile, APProfile> profileModifier) {
        APProfile modifiedProfile = profileModifier.apply(m_profile);
        return new AutoAlign(
                m_target,
                m_drivetrain,
                modifiedProfile,
                m_rotationControlMode,
                m_rotationConstraints);
    }

    public AutoAlign withVelocityLimitedRotation() {
        return withRotationControlMode(RotationControlMode.VELOCITY_LIMITED_PROFILE);
    }

    public AutoAlign withVelocityLimitedRotation(RotationConstraints rotationConstraints) {
        return new AutoAlign(
                m_target,
                m_drivetrain,
                m_profile,
                RotationControlMode.VELOCITY_LIMITED_PROFILE,
                rotationConstraints);
    }

    public AutoAlign withRotationControlMode(RotationControlMode rotationControlMode) {
        return new AutoAlign(
                m_target,
                m_drivetrain,
                m_profile,
                rotationControlMode,
                m_rotationConstraints);
    }

    /**
     * Gets the current profile being used
     */
    public APProfile getProfile() {
        return m_profile;
    }

    public RotationControlMode getRotationControlMode() {
        return m_rotationControlMode;
    }

    @Override
    public void initialize() {
        swerveState = m_drivetrain.getState();
        double maxVelocity = Math.max(0, m_rotationConstraints.maxVelocity);
        m_rotationSetpointRadians = swerveState.Pose.getRotation().getRadians();
        m_rotationSetpointVelocity = MathUtil.clamp(
                swerveState.Speeds.omegaRadiansPerSecond,
                -maxVelocity,
                maxVelocity);
        m_lastRotationProfileTimestamp = swerveState.Timestamp;
    }

    @Override
    public void execute() {
        swerveState = m_drivetrain.getState();
        APResult out = kAutopilot.calculate(swerveState.Pose, swerveState.Speeds, m_target);

        applyDriveRequest(out);
    }

    protected void applyDriveRequest(APResult out) {
        if (m_rotationControlMode == RotationControlMode.VELOCITY_LIMITED_PROFILE) {
            applyVelocityLimitedRotationRequest(out);
            return;
        }

        m_drivetrain.setControl(m_request
                .withVelocityX(out.vx())
                .withVelocityY(out.vy())
                .withTargetRateFeedforward(0)
                .withMaxAbsRotationalRate(0)
                .withTargetDirection(out.targetAngle()));
    }

    private void applyVelocityLimitedRotationRequest(APResult out) {
        calculateRotationSetpoint(out.targetAngle());

        double maxVelocity = Math.max(0, m_rotationConstraints.maxVelocity);
        m_drivetrain.setControl(m_request
                .withVelocityX(out.vx())
                .withVelocityY(out.vy())
                .withTargetDirection(Rotation2d.fromRadians(m_rotationSetpointRadians))
                .withTargetRateFeedforward(m_rotationSetpointVelocity)
                .withMaxAbsRotationalRate(maxVelocity));
    }

    private void calculateRotationSetpoint(Rotation2d targetAngle) {
        double dt = swerveState.Timestamp - m_lastRotationProfileTimestamp;
        m_lastRotationProfileTimestamp = swerveState.Timestamp;

        if (dt <= 0) {
            dt = AutoAlignConstants.ROTATION_PROFILE_PERIOD;
        }
        dt = MathUtil.clamp(dt, AutoAlignConstants.ROTATION_PROFILE_PERIOD, AutoAlignConstants.ROTATION_PROFILE_MAX_PERIOD);

        double goalRadians = m_rotationSetpointRadians
                + MathUtil.inputModulus(
                        targetAngle.getRadians() - m_rotationSetpointRadians,
                        -Math.PI,
                        Math.PI);

        double maxVelocity = Math.max(0, m_rotationConstraints.maxVelocity);
        double maxAcceleration = Math.max(0, m_rotationConstraints.maxAcceleration);
        double error = goalRadians - m_rotationSetpointRadians;
        double desiredVelocity = 0;
        if (Math.abs(error) > 1e-6 && maxVelocity > 0 && maxAcceleration > 0) {
            double maxVelocityForStopping = Math.sqrt(
                    2 * maxAcceleration * Math.abs(error));
            desiredVelocity = Math.copySign(
                    Math.min(maxVelocity, maxVelocityForStopping),
                    error);
        }

        m_rotationSetpointVelocity = MathUtil.clamp(
                desiredVelocity,
                m_rotationSetpointVelocity - maxAcceleration * dt,
                m_rotationSetpointVelocity + maxAcceleration * dt);
        m_rotationSetpointRadians += m_rotationSetpointVelocity * dt;

        if (Math.signum(goalRadians - m_rotationSetpointRadians) != Math.signum(error)) {
            m_rotationSetpointRadians = goalRadians;
            m_rotationSetpointVelocity = 0;
        }
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
