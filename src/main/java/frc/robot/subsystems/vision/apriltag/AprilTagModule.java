package frc.robot.subsystems.vision.apriltag;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.apriltag.RealATVision.ATVisionConstants;
import frc.robot.util.LimelightHelpers;

/**
 * Wrapper class for a single Limelight. Records vision data to NetworkTables for debugging.
 */
public class AprilTagModule {

    public static record AprilTagEstimate(Pose2d estimatedPose, double timestampSeconds, boolean isMegaTag2, double avgTagDistMeters, int tagCount, double avgAmbiguity, double tagArea) {}

    public enum EstimationMode {
        MEGATAG1,
        MEGATAG2;
    }

    private final String limelightID;

    private final NetworkTable moduleSubTable;

    private final StructPublisher<Pose3d> robotToCameraPublisher;
    private final StructPublisher<Pose3d> estimatePublisher;
    private final BooleanPublisher isActivePublisher;
    private final BooleanPublisher isConnectedPublisher;
    private final StringPublisher modePublisher;
    private final StringPublisher defaultModePublisher;

    private final EstimationMode defaultMode;
    private EstimationMode lastMode;

    private double lastHb = Double.NaN;
    private double lastHbChangeTimestamp;
    private boolean connected = false;
    private boolean wasConnected = false;
    private double lastDisconnectReportTimestamp = Double.NEGATIVE_INFINITY;

    /** Cached result of this loop's read, so telemetry and consumers share one NT round trip. */
    private Optional<AprilTagEstimate> latestEstimate = Optional.empty();

    public AprilTagModule(String limelightID, Pose3d offset, NetworkTable visionTable) {
        this.limelightID = limelightID;

        defaultMode = ATVisionConstants.kDefaultMode;
        lastMode = defaultMode;
        lastHbChangeTimestamp = Timer.getFPGATimestamp();

            // Publishers for Limelight data
        moduleSubTable = visionTable.getSubTable(limelightID);
        robotToCameraPublisher = moduleSubTable.getStructTopic("CameraOffset", Pose3d.struct).publish();
        estimatePublisher = moduleSubTable.getStructTopic("PoseEstimate", Pose3d.struct).publish();
        isActivePublisher = moduleSubTable.getBooleanTopic("IsActive").publish();
        modePublisher = moduleSubTable.getStringTopic("LastEstimateMode").publish();
        defaultModePublisher = moduleSubTable.getStringTopic("DefaultEstimateMode").publish();
        isConnectedPublisher = moduleSubTable.getBooleanTopic("IsConnected").publish();

        applyConfig();
        updateOffset(offset);
        defaultModePublisher.setDefault(defaultMode.name());
    }

    /**
     * Pushes camera-side settings. Re-applied whenever the Limelight reconnects, since a camera
     * that boots after the RIO never sees the settings written at construction time.
     */
    private void applyConfig() {
        // 0 = use the externally supplied robot orientation (robot_orientation_set) for MegaTag2.
        LimelightHelpers.SetIMUMode(limelightID, 0);
        // Must go to the Limelight's own NT table -- writing "rewind_enable_set" into our
        // telemetry subtable never reaches the camera.
        LimelightHelpers.setRewindEnabled(limelightID, true);
    }

    /**
     * Must be called periodically in {@link frc.robot.subsystems.vision.apriltag.RealATVision#periodic()}.
     *
     * @param mode Which MegaTag solver to read this loop.
     */
    public void periodic(EstimationMode mode) {
        double now = Timer.getFPGATimestamp();

        double hb = LimelightHelpers.getHeartbeat(limelightID);
        if (hb != lastHb) {
            lastHb = hb;
            lastHbChangeTimestamp = now;
        }
        // Allow for pipelines running slower than the 50 Hz robot loop -- comparing the heartbeat
        // to only the previous loop's value reports a spurious disconnect on every repeat frame.
        connected = (now - lastHbChangeTimestamp) < ATVisionConstants.kHeartbeatTimeoutSeconds;

        if (connected && !wasConnected) {
            applyConfig();
        }
        wasConnected = connected;

        if (!connected && (now - lastDisconnectReportTimestamp) >= ATVisionConstants.kDisconnectReportPeriodSeconds) {
            lastDisconnectReportTimestamp = now;
            DriverStation.reportError(limelightID + " is not connected.", false);
        }

        latestEstimate = readPose(mode == EstimationMode.MEGATAG2);

        updateTelemetry();
    }

    /** @return This loop's pose estimate, or empty if the Limelight had nothing usable. */
    public Optional<AprilTagEstimate> getLatestEstimate() {
        return latestEstimate;
    }

    /**
     * Updates the {@link edu.wpi.first.networktables.NetworkTable NetworkTable} subtable for the Limelight.
     * Records the latest pose estimate, whether or not the Limelight has estimate data, the
     * {@link EstimationMode} last read, and the default {@link EstimationMode}.
     */
    private void updateTelemetry() {
        estimatePublisher.accept(latestEstimate.map(e -> new Pose3d(e.estimatedPose())).orElse(Pose3d.kZero));
        isActivePublisher.accept(latestEstimate.isPresent());
        modePublisher.accept(lastMode.toString());
        defaultModePublisher.accept(defaultMode.toString());
        isConnectedPublisher.accept(connected);
    }

    public Pose3d getOffset() {
        return LimelightHelpers.getCameraPose3d_RobotSpace(limelightID);
    }

    /**
     * Pushes the robot-to-camera transform to the Limelight.
     *
     * @param offset Robot-to-camera pose in the WPILib frame (+X fwd, +Y left, +Z up).
     * @see ATVisionConstants#kLimelightRobotSpaceIsYRight
     */
    public void updateOffset(Pose3d offset) {
        Rotation3d cameraRot = offset.getRotation();
        double handedness = ATVisionConstants.kLimelightRobotSpaceIsYRight ? -1.0 : 1.0;
        LimelightHelpers.setCameraPose_RobotSpace(
            limelightID,
            offset.getX(),
            handedness * offset.getY(),
            offset.getZ(),
            Math.toDegrees(cameraRot.getX()),
            handedness * Math.toDegrees(cameraRot.getY()),
            handedness * Math.toDegrees(cameraRot.getZ())
        );
        robotToCameraPublisher.accept(offset);
    }

    /**
     * Checks whether the Limelight's heartbeat has advanced recently.
     *
     * @return Whether or not the Limelight is connected
     */
    public boolean isConnected() {
        return connected;
    }

    public boolean hasTargets() {
        return LimelightHelpers.getTV(limelightID);
    }

    /**
     * Reads a MegaTag pose estimate. Returns {@link java.util.Optional#empty()} if the Limelight
     * has no tags in view or the botpose array is malformed.
     *
     * @param isMegaTag2 Whether to read the MegaTag2 (orb) solve rather than MegaTag1.
     * @return The estimated pose if the Limelight has targets
     */
    private Optional<AprilTagEstimate> readPose(boolean isMegaTag2) {
        DoubleArrayEntry poseEntry = LimelightHelpers.getLimelightDoubleArrayEntry(limelightID, isMegaTag2 ? "botpose_orb_wpiblue" : "botpose_wpiblue");
        TimestampedDoubleArray tsValue = poseEntry.getAtomic();
        double[] poseArray = tsValue.value;
        long timestamp = tsValue.timestamp;

        if (poseArray.length == 0) {
            // Handle the case where no data is available
            return Optional.empty();
        }

        double latency = LimelightHelpers.extractArrayEntry(poseArray, 6);
        int tagCount = (int) LimelightHelpers.extractArrayEntry(poseArray, 7);
        // double tagSpan = LimelightHelpers.extractArrayEntry(poseArray, 8);
        double tagDist = LimelightHelpers.extractArrayEntry(poseArray, 9);
        double tagArea = LimelightHelpers.extractArrayEntry(poseArray, 10);

        // With no tags the array is a valid, all-zero, 11-length array. Reject it here rather than
        // letting a (0,0,0) pose through and dividing by tagCount below.
        if (tagCount <= 0) {
            return Optional.empty();
        }

        final int valsPerFiducial = 7;
        if (poseArray.length != 11 + valsPerFiducial * tagCount) {
            // Array size mismatch - don't try to read per-tag data that isn't there
            return Optional.empty();
        }

        double avgAmbiguity = 0;
        for (int i = 0; i < tagCount; i++) {
            int baseIndex = 11 + (i * valsPerFiducial);
            avgAmbiguity += poseArray[baseIndex + 6];
        }
        avgAmbiguity /= tagCount;

        var pose = LimelightHelpers.toPose2D(poseArray);

        // Convert server timestamp from microseconds to seconds and adjust for latency
        double adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);

        lastMode = isMegaTag2 ? EstimationMode.MEGATAG2 : EstimationMode.MEGATAG1;
        return Optional.of(new AprilTagEstimate(pose, adjustedTimestamp, isMegaTag2, tagDist, tagCount, avgAmbiguity, tagArea));
    }

    /**
     * Seeds the robot orientation used by MegaTag2. Does not flush -- the caller is expected to
     * flush once per loop after seeding every camera.
     *
     * @param rot Field-relative robot orientation.
     */
    public void seedOrientation(Rotation3d rot) {
        // LimelightHelpers takes DEGREES; Rotation3d getters return radians.
        LimelightHelpers.SetRobotOrientation_NoFlush(
            limelightID,
            Math.toDegrees(rot.getZ()),
            0,
            Math.toDegrees(rot.getY()),
            0,
            Math.toDegrees(rot.getX()),
            0
        );
    }

    public void captureRewind(double seconds) {
        LimelightHelpers.triggerRewindCapture(limelightID, seconds);
    }
}
