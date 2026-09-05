package frc.robot.subsystems.vision.apriltag;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.turret.Turret.TurretConstants;
import frc.robot.subsystems.vision.apriltag.AprilTagModule.AprilTagEstimate;
import frc.robot.subsystems.vision.apriltag.AprilTagModule.EstimationMode;
import frc.robot.util.LimelightHelpers;

public class RealATLimelightVision extends AprilTagVision {
    public static class ATVisionConstants {
        public static final String[] LL_IDS = {
            "limelight-turret"
        };

        /**
         * Initial robot-to-camera pose per Limelight. The turret camera's entry is recomputed from
         * the live turret angle every loop by {@link RealATVision#periodic()}; this is only what
         * gets pushed before the first loop runs.
         */
        public static final Pose3d[] LL_OFFSETS = {
            solveRobotToCamera(0.0)
        };

        /** Index into {@link #LL_IDS} of the turret-mounted camera. */
        public static final int kTurretCameraIndex = 0;

        public static final EstimationMode kDefaultMode = EstimationMode.MEGATAG1;

        public static final double[] kMT2StdDevCoefficients = {0.085, 0.0}; // deviation order is [xy, theta]
        public static final double[] kMT1StdDevCoefficients = {0.1, 0.075};
        public static final int kOptimalTagCount = 2;

        /**
         * Whether the Limelight's camerapose_robotspace frame is +Y right (making it mirrored
         * relative to WPILib's +Y left), which also flips the sign of roll and yaw.
         * <p>
         * VERIFY THIS ON THE FIELD BEFORE TRUSTING OFF-AXIS TURRET ANGLES. With the turret at 0 deg
         * the camera is on the centerline with zero yaw, so both settings are identical and the
         * error is invisible. Test: park the robot at a known pose with a tag in view, rotate the
         * turret to +90 deg, and watch the botpose estimate. If it stays put, this value is right;
         * if it swings off by roughly twice the camera's offset, flip it.
         * <p>
         * Left at {@code false} (no conversion) to match the behaviour this code had before the
         * turret angle was wired in.
         */
        public static final boolean kLimelightRobotSpaceIsYRight = false;

        /** How long the heartbeat may stall before the camera counts as disconnected. */
        public static final double kHeartbeatTimeoutSeconds = 0.25;
        /** Minimum spacing between repeated "not connected" driver station errors. */
        public static final double kDisconnectReportPeriodSeconds = 3.0;

        /** Reject estimates above this ambiguity. */
        public static final double kMaxAmbiguity = 0.3;
        /** Reject estimates while the chassis is yawing faster than this. */
        public static final double kMaxChassisOmegaRadPerSec = Math.PI / 2;
        /** Reject estimates while the robot is tilted more than this (in radians). */
        public static final double kMaxTiltRad = Math.toRadians(20);

        /**
         * How much turret angle / chassis motion history to keep. Must comfortably exceed
         * {@link #kMaxEstimateAgeSeconds} so the history actually spans every estimate we consider.
         */
        public static final double kEstimateHistorySeconds = 1.0;

        /** Reject estimates captured longer ago than this. */
        public static final double kMaxEstimateAgeSeconds = 0.4;

        /** Slack for clock jitter when an estimate's timestamp lands slightly in the future. */
        public static final double kClockSkewToleranceSeconds = 0.01;

        /**
         * Delay between writing camerapose_robotspace and the Limelight actually solving a frame
         * with it. This is the one number that makes the geometry check meaningful, and it is
         * empirical -- the Limelight does not tell us when a written offset took effect.
         * <p>
         * {@code Vision/TurretMismatchDeg} is a PREDICTION derived from this constant -- it works
         * out to roughly (turret rate) x (this value) -- not a measurement of real error. Do not
         * tune this by minimising it; that would just drive it to zero.
         * <p>
         * Determine it against ground truth instead: park the robot at a surveyed pose with a tag
         * in view, slew the turret at several constant rates, and record the heading error of the
         * reported pose at each rate. That error grows as (turret rate) x (true lag), so the slope
         * of heading error against turret rate is the lag in seconds.
         * <p>
         * Too small and real mismatch is under-reported (bad poses get through); too large and
         * good frames get rejected.
         */
        public static final double kOffsetTransportLatencySeconds = 0.02;

        /**
         * Reject a turret-camera estimate when the camera geometry the Limelight used differs from
         * where the turret actually was at capture time by more than this.
         * <p>
         * This is the real gate on turret motion: it is expressed in the units that matter
         * (degrees of geometric error) and evaluated at the moment the frame was captured, rather
         * than testing the turret's velocity right now -- which is the wrong instant, and would
         * happily accept a frame captured mid-slew just because the turret has since stopped.
         */
        public static final double kMaxTurretMismatchDeg = 3.0;

        /**
         * Reject a turret-camera estimate captured while the turret was slewing faster than this.
         * <p>
         * Separate from {@link #kMaxTurretMismatchDeg}: that one covers geometry we got wrong,
         * this one covers motion blur, which corrupts the image no matter how well we know where
         * the camera was pointing. Deliberately loose -- the mismatch check does the real work.
         */
        public static final double kMaxTurretVelDegPerSec = 180.0;

        /** Interval used to difference the turret history into a velocity. */
        public static final double kVelocitySampleDtSeconds = 0.02;
    }

    private final AprilTagModule[] limelights;
    /** The turret-mounted camera, or null if no cameras are configured. */
    private final AprilTagModule turretCamera;

    private final Supplier<SwerveDriveState> swerveState;
    private final Supplier<Rotation3d> gyroRotation;
    private final BiConsumer<AprilTagEstimate, Matrix<N3,N1>> addVisionMeasurement;
    private final Supplier<Double> turretAngleSupplier;

    /** Where the turret actually was, by FPGA timestamp. */
    private final TimeInterpolatableBuffer<Rotation2d> turretAngleBuffer =
        TimeInterpolatableBuffer.createBuffer(ATVisionConstants.kEstimateHistorySeconds);
    /** Which turret angle we pushed to the Limelight, by the FPGA timestamp of the write. */
    private final TimeInterpolatableBuffer<Rotation2d> pushedAngleBuffer =
        TimeInterpolatableBuffer.createBuffer(ATVisionConstants.kEstimateHistorySeconds);
    /**
     * Chassis yaw rate history, by FPGA timestamp. Looked up at each estimate's capture time
     * rather than read fresh -- same reasoning as the turret mismatch check below: testing "right
     * now" would happily accept a frame captured mid-spin just because the chassis has since
     * settled, and would just as wrongly reject a good frame captured while stationary because the
     * chassis happens to be spinning now.
     */
    private final TimeInterpolatableBuffer<Double> chassisOmegaBuffer =
        TimeInterpolatableBuffer.createDoubleBuffer(ATVisionConstants.kEstimateHistorySeconds);
    /** Gyro roll history, by FPGA timestamp. See {@link #chassisOmegaBuffer}. */
    private final TimeInterpolatableBuffer<Double> rollBuffer =
        TimeInterpolatableBuffer.createDoubleBuffer(ATVisionConstants.kEstimateHistorySeconds);
    /** Gyro pitch history, by FPGA timestamp. See {@link #chassisOmegaBuffer}. */
    private final TimeInterpolatableBuffer<Double> pitchBuffer =
        TimeInterpolatableBuffer.createDoubleBuffer(ATVisionConstants.kEstimateHistorySeconds);

    private final NetworkTable visionTable;

    private boolean headingSeeded = false;

    private final BooleanPublisher headingSeededPublisher;
    private final StructPublisher<Pose3d> seededPosePublisher;
    private final StructPublisher<Pose3d> robotToCameraPublisher;
    private final DoublePublisher acceptedCountPublisher;
    private final DoublePublisher mismatchPublisher;
    private final DoublePublisher estimateAgePublisher;
    private final StringPublisher rejectReasonPublisher;

    /** Diagnostics for the most recently examined estimate. Tuning aids, not control inputs. */
    private double lastMismatchDeg = 0;
    private double lastAgeSeconds = 0;
    private String lastRejectReason = "";

    public RealATVision(
            Supplier<SwerveDriveState> swerveState,
            Supplier<Rotation3d> gyroRotation,
            BiConsumer<AprilTagEstimate, Matrix<N3,N1>> addVisionMeasurement,
            Supplier<Double> turretAngleSupplier) {
        this.swerveState = swerveState;
        this.gyroRotation = gyroRotation;
        this.addVisionMeasurement = addVisionMeasurement;
        this.turretAngleSupplier = turretAngleSupplier;
        limelights = new AprilTagModule[ATVisionConstants.LL_IDS.length];

        visionTable = NetworkTableInstance.getDefault().getTable("Vision");
        headingSeededPublisher = visionTable.getBooleanTopic("HeadingSeeded").publish();
        seededPosePublisher = visionTable.getStructTopic("SeededPose", Pose3d.struct).publish();
        robotToCameraPublisher = visionTable.getStructTopic("TurretRobotToCamera", Pose3d.struct).publish();
        acceptedCountPublisher = visionTable.getDoubleTopic("AcceptedEstimates").publish();
        mismatchPublisher = visionTable.getDoubleTopic("TurretMismatchDeg").publish();
        estimateAgePublisher = visionTable.getDoubleTopic("EstimateAgeSeconds").publish();
        rejectReasonPublisher = visionTable.getStringTopic("LastRejectReason").publish();

        for(int i = 0; i < limelights.length; i++) {
            limelights[i] = new AprilTagModule(ATVisionConstants.LL_IDS[i], ATVisionConstants.LL_OFFSETS[i], visionTable);
        }

        turretCamera = (ATVisionConstants.kTurretCameraIndex < limelights.length)
            ? limelights[ATVisionConstants.kTurretCameraIndex]
            : null;
    }

    /**
     * Composes the robot-to-camera transform for a turret-mounted camera:
     * robot -> turret axis -> turret rotation -> camera.
     *
     * @param turretAngleDeg Turret angle in degrees, CCW positive about the robot's +Z.
     * @return Robot-to-camera pose in the WPILib frame.
     */
    public static Pose3d solveRobotToCamera(double turretAngleDeg) {
        Rotation3d turretRotation = new Rotation3d(0, 0, Math.toRadians(turretAngleDeg));

        // Rotate the turret frame in place, then step out to the camera within that frame. This
        // keeps the turret's height: rotating the camera pose about the turret centre with
        // Pose3d#rotateAround cancels the Z offset out entirely and puts the camera on the floor.
        Pose3d rotatedTurret = new Pose3d(
            TurretConstants.turretCenterPose.getTranslation(),
            TurretConstants.turretCenterPose.getRotation().rotateBy(turretRotation));

        return rotatedTurret.transformBy(new Transform3d(
            TurretConstants.CAMERA_POSE3D.getTranslation(),
            TurretConstants.CAMERA_POSE3D.getRotation()));
    }

    @Override
    public void periodic() {
        estimates.clear();

        // Same clock the Limelight estimate timestamps end up on: NT server time on the RIO shares
        // the FPGA time base, and AprilTagModule converts to seconds without rebasing. Keep the
        // turret history on this clock so the two are directly comparable.
        double now = Timer.getFPGATimestamp();

        SwerveDriveState state = swerveState.get();
        // The Pigeon gives usable roll/pitch, but its yaw is boot-relative and is not updated by
        // resetPose -- MegaTag2 needs field-relative yaw, which only the pose estimator has.
        Rotation3d gyroRot = gyroRotation.get();
        Rotation2d fieldYaw = state.Pose.getRotation();
        Rotation3d seedRotation = new Rotation3d(gyroRot.getX(), gyroRot.getY(), fieldYaw.getRadians());

        chassisOmegaBuffer.addSample(now, state.Speeds.omegaRadiansPerSecond);
        rollBuffer.addSample(now, gyroRot.getX());
        pitchBuffer.addSample(now, gyroRot.getY());

        Rotation2d turretAngle = Rotation2d.fromDegrees(turretAngleSupplier.get());
        turretAngleBuffer.addSample(now, turretAngle);

        if (turretCamera != null) {
            Pose3d robotToCamera = solveRobotToCamera(turretAngle.getDegrees());
            turretCamera.updateOffset(robotToCamera);
            // Record what we sent, not just where the turret was, so the geometry check stays
            // honest if pushing ever becomes conditional.
            pushedAngleBuffer.addSample(now, turretAngle);
            robotToCameraPublisher.accept(robotToCamera);
        }

        // MegaTag2 is only as good as the yaw we hand it, so stay on MegaTag1 until a measurement
        // has actually been applied to the estimator. Also prefer MegaTag1 while disabled, where
        // its independent yaw solve is what pulls the estimator onto the field.
        boolean useMegaTag1 = DriverStation.isDisabled() || !headingSeeded;
        EstimationMode mode = useMegaTag1 ? EstimationMode.MEGATAG1 : EstimationMode.MEGATAG2;

        int accepted = 0;
        for (int i = 0; i < limelights.length; i++) {
            AprilTagModule limelight = limelights[i];

            // Seed unconditionally, even on MegaTag1 loops. MegaTag1 ignores it, and it means the
            // camera already has a current orientation the moment we switch to MegaTag2 rather
            // than solving one frame against a stale (or absent) one.
            limelight.seedOrientation(seedRotation);
            limelight.periodic(mode);

            var estOpt = limelight.getLatestEstimate();
            if (estOpt.isEmpty()) continue;
            AprilTagEstimate estimate = estOpt.get();
            estimates.add(estimate);

            boolean isTurretCam = (i == ATVisionConstants.kTurretCameraIndex);

            // Record the turret diagnostics for every estimate that arrives, independently of the
            // accept/reject decision -- these are what kOffsetTransportLatencySeconds is tuned
            // against, so they must not go stale just because some other gate fired first.
            if (isTurretCam) {
                lastAgeSeconds = now - estimate.timestampSeconds();
                if (isFresh(estimate.timestampSeconds(), now)) {
                    turretMismatchDeg(estimate.timestampSeconds())
                        .ifPresent(mismatch -> lastMismatchDeg = mismatch);
                }
            }

            String reject = rejectReason(estimate, now, isTurretCam);
            if (!reject.isEmpty()) {
                lastRejectReason = reject;
                continue;
            }

            addVisionMeasurement.accept(estimate, DriverStation.isEnabled()
                ? AprilTagVision.getStdDevs(estimate)
                : AprilTagVision.getDisabledStdDevs(estimate));
            accepted++;
            lastRejectReason = "";
            // Only claim the heading is seeded once a measurement has actually been applied --
            // otherwise a rejected estimate hands MegaTag2 a yaw that was never corrected.
            headingSeeded = true;
        }

        if (limelights.length > 0) {
            // seedOrientation deliberately doesn't flush; push every camera's orientation at once.
            LimelightHelpers.Flush();
        }

        headingSeededPublisher.accept(headingSeeded);
        // The orientation handed to MegaTag2 -- rotation only, translation is meaningless here.
        seededPosePublisher.accept(new Pose3d(Translation3d.kZero, seedRotation));
        acceptedCountPublisher.accept(accepted);
        mismatchPublisher.accept(lastMismatchDeg);
        estimateAgePublisher.accept(lastAgeSeconds);
        rejectReasonPublisher.accept(lastRejectReason);
    }

    /**
     * Decides whether an estimate is worth handing to the pose estimator.
     *
     * @param isTurretCam Whether this estimate came from the turret-mounted camera. Fixed cameras
     *                    skip the turret geometry checks entirely -- their offset never moves.
     * @return An empty string to accept, otherwise a short reason the estimate was rejected.
     */
    private String rejectReason(AprilTagEstimate est, double now, boolean isTurretCam) {
        // Freshness applies to EVERY camera, not just the turret one: a disconnected Limelight
        // leaves its last botpose sitting in NetworkTables, so without this we would re-accept the
        // same stale pose every loop for as long as it stayed down. It must also come before any
        // buffer lookup below -- see isFresh's docs on why an out-of-range timestamp can't be
        // trusted to report a miss.
        double captureTime = est.timestampSeconds();
        if (now - captureTime < -ATVisionConstants.kClockSkewToleranceSeconds) return "timestamp in the future";
        if (!isFresh(captureTime, now)) return "estimate too old";

        // Evaluated at the moment the frame was captured, not "right now" -- the wrong instant
        // would happily accept a frame captured mid-spin just because the chassis has since
        // settled, and just as wrongly reject a good frame captured while stationary because the
        // chassis happens to be moving now. Same reasoning as the turret checks below.
        Optional<Double> chassisOmega = chassisOmegaBuffer.getSample(captureTime);
        Optional<Double> roll = rollBuffer.getSample(captureTime);
        Optional<Double> pitch = pitchBuffer.getSample(captureTime);
        if (chassisOmega.isEmpty() || roll.isEmpty() || pitch.isEmpty()) return "no chassis history";
        if (Math.abs(chassisOmega.get()) >= ATVisionConstants.kMaxChassisOmegaRadPerSec) return "chassis yaw rate";
        if (Math.abs(roll.get()) >= ATVisionConstants.kMaxTiltRad
            || Math.abs(pitch.get()) >= ATVisionConstants.kMaxTiltRad) return "robot tilted";

        if (est.tagCount() <= 0) return "no tags";
        if (est.avgAmbiguity() > ATVisionConstants.kMaxAmbiguity) return "ambiguity";
        // A pose sitting on the field origin means the solve produced nothing useful.
        if (est.estimatedPose().getTranslation().getDistance(Translation2d.kZero) < 0.05) return "pose at origin";

        if (!isTurretCam) return "";

        var mismatch = turretMismatchDeg(captureTime);
        var velocity = turretVelAtCaptureDegPerSec(captureTime);
        if (mismatch.isEmpty() || velocity.isEmpty()) return "no turret history";

        if (mismatch.getAsDouble() > ATVisionConstants.kMaxTurretMismatchDeg) return "turret geometry mismatch";
        if (Math.abs(velocity.getAsDouble()) > ATVisionConstants.kMaxTurretVelDegPerSec) return "turret slewing";

        return "";
    }

    /**
     * Whether an estimate captured at the given time is recent enough to use.
     * <p>
     * Serves two purposes. It rejects stale poses generally -- a Limelight that drops off the
     * network leaves its last botpose in NetworkTables, where it would otherwise look valid
     * forever. And because the bound is well inside {@link ATVisionConstants#kEstimateHistorySeconds},
     * a fresh estimate is also guaranteed to be spanned by the chassis/tilt/turret history buffers:
     * relevant because {@link TimeInterpolatableBuffer#getSample(double)} clamps to the ends of its
     * history rather than reporting a miss, so an out-of-range timestamp silently returns the
     * oldest or newest sample instead of nothing. Every buffer lookup must be bounded by this first.
     */
    private boolean isFresh(double captureTime, double now) {
        double age = now - captureTime;
        return age >= -ATVisionConstants.kClockSkewToleranceSeconds
            && age <= ATVisionConstants.kMaxEstimateAgeSeconds;
    }

    /**
     * How far the camera geometry the Limelight used differs from where the turret actually was
     * when the frame was captured.
     *
     * @param captureTime FPGA timestamp of the frame. Only meaningful if {@link #isFresh}.
     * @return Absolute geometry error in degrees, or empty if the history has no samples.
     */
    private OptionalDouble turretMismatchDeg(double captureTime) {
        Optional<Rotation2d> actual = turretAngleBuffer.getSample(captureTime);
        Optional<Rotation2d> used = pushedAngleBuffer.getSample(
            captureTime - ATVisionConstants.kOffsetTransportLatencySeconds);
        if (actual.isEmpty() || used.isEmpty()) return OptionalDouble.empty();
        // Rotation2d#minus wraps to (-180, 180], which is what we want here: a camera at 0 deg and
        // one at 360 deg are the same camera.
        return OptionalDouble.of(Math.abs(actual.get().minus(used.get()).getDegrees()));
    }

    /**
     * Turret angular velocity at the moment a frame was captured, differenced from the history.
     *
     * @param captureTime FPGA timestamp of the frame. Only meaningful if {@link #isFresh}.
     * @return Turret rate in degrees per second, or empty if the history has no samples.
     */
    private OptionalDouble turretVelAtCaptureDegPerSec(double captureTime) {
        Optional<Rotation2d> actual = turretAngleBuffer.getSample(captureTime);
        Optional<Rotation2d> prior = turretAngleBuffer.getSample(
            captureTime - ATVisionConstants.kVelocitySampleDtSeconds);
        if (actual.isEmpty() || prior.isEmpty()) return OptionalDouble.empty();
        return OptionalDouble.of(actual.get().minus(prior.get()).getDegrees()
            / ATVisionConstants.kVelocitySampleDtSeconds);
    }

    @Override
    public void updateOffsets(Pose3d[] offsets) {
        if(offsets.length != limelights.length) return;
        for(int i = 0; i < limelights.length; i++) {
            // The turret camera's offset is recomputed from the live turret angle every loop, so
            // writing to it here would just be overwritten (and fight the solve).
            if(offsets[i] == null || i == ATVisionConstants.kTurretCameraIndex) continue;
            limelights[i].updateOffset(offsets[i]);
        }
    }

    @Override
    protected void captureRewinds(double seconds) {
        for(AprilTagModule cam : limelights) {
            cam.captureRewind(seconds);
        }
    }
}
