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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlign extends Command {
    // Static factory methods with profile parameters


    public static class AutoAlignConstants {
        public static double DEFAULT_MAX_VELOCITY = 5.5; // physical max is 5.5 m/s^2
        public static double DEFAULT_ACCELERATION = 23; // Calculated from swerve slip current
        public static double DEFAULT_JERK = 6.0;

        // Constants are listed as (velocity, acceleration, jerk) or (acceleration,
        // jerk)
        public static APConstraints SLOW_DRIVE_CONSTRAINTS = new APConstraints(1.3, DEFAULT_ACCELERATION, 20);
                public static APConstraints SLOW_CRAWL_CONSTRAINTS = new APConstraints(0.5, DEFAULT_ACCELERATION, 20);

        public static APConstraints VELOCITY_LIMITED_CONSTRAINTS = new APConstraints(DEFAULT_MAX_VELOCITY, DEFAULT_ACCELERATION, DEFAULT_JERK);
        public static APConstraints HIGH_JERK_CONSTRAINTS = new APConstraints(DEFAULT_MAX_VELOCITY, DEFAULT_ACCELERATION, 60);
        public static APConstraints DEFAULT_CONSTRAINTS = new APConstraints(DEFAULT_ACCELERATION, DEFAULT_JERK);
        public static APConstraints CLIMB_CONSTRAINTS = new APConstraints(20, 3);
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

    public static APProfile kGPDProfile = new APProfile(
            AutoAlignConstants.SLOW_DRIVE_CONSTRAINTS)
            .withErrorXY(Meters.of(0.2))
            .withErrorTheta(Degrees.of(10))
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
    protected final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric();
    protected final SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withHeadingPID(5, 0, 0); // Replace with constants later

    protected SwerveDriveState swerveState = new SwerveDriveState();

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

    public AutoAlign(Pose2d targetPose, CommandSwerveDrivetrain drivetrain, APProfile profile) {
        this(new APTarget(targetPose), drivetrain, profile);
    }

    /**
     * Auto align constructor with full parameters
     * 
     * @param target     APTarget to align to
     * @param drivetrain Drivetrain subsystem
     * @param profile    APProfile to use for this alignment
     */
    public AutoAlign(APTarget target, CommandSwerveDrivetrain drivetrain, APProfile profile) {
        this.m_target = target;
        this.m_drivetrain = drivetrain;
        this.m_profile = profile;

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
        return new AutoAlign(m_target, m_drivetrain, modifiedProfile);
    }

    /**
     * Gets the current profile being used
     */
    public APProfile getProfile() {
        return m_profile;
    }

    @Override
    public void execute() {
        swerveState = m_drivetrain.getState();
        APResult out = kAutopilot.calculate(swerveState.Pose, swerveState.Speeds, m_target);

        m_drivetrain.setControl(m_request
                .withVelocityX(out.vx())
                .withVelocityY(out.vy())
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