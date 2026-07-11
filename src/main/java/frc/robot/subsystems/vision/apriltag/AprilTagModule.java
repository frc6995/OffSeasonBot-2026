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
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.apriltag.RealATVision.ATVisionConstants;
import frc.robot.util.LimelightHelpers;

/**
 * Wrapper class for a Yet Another Limelight Library {@link limelight.Limelight} object. 
 * Records vision data to NetworkTables for debugging. 
 */
public class AprilTagModule {
    public static record AprilTagEstimate(Pose2d estimatedPose, double timestampSeconds, boolean isMegaTag2, double avgTagDistMeters, double tagCount, double avgAmbiguity) {}
    
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

    private EstimationMode defaultMode;
    private EstimationMode lastMode;

    private double hb = 0;
    private double lastHb = 0;

    public AprilTagModule(String limelightID, Pose3d offset, NetworkTable visionTable) {
        this.limelightID = limelightID;

        LimelightHelpers.SetIMUMode(limelightID, 0);
        defaultMode = ATVisionConstants.kDefaultMode;

            // Publishers for Limelight data
        moduleSubTable = visionTable.getSubTable(limelightID);
        robotToCameraPublisher = moduleSubTable.getStructTopic("CameraOffset", Pose3d.struct).publish();
        estimatePublisher = moduleSubTable.getStructTopic("PoseEstimate", Pose3d.struct).publish();
        isActivePublisher = moduleSubTable.getBooleanTopic("IsActive").publish();
        modePublisher = moduleSubTable.getStringTopic("LastEstimateMode").publish();
        defaultModePublisher = moduleSubTable.getStringTopic("DefaultEstimateMode").publish();
        isConnectedPublisher = moduleSubTable.getBooleanTopic("IsConnected").publish();

        // setDouble(1) to enable rewind, 0 to disable
        moduleSubTable.getEntry("rewind_enable_set").setDouble(1);
        robotToCameraPublisher.accept(offset);
        defaultModePublisher.setDefault(defaultMode.name());
    }

    /**
     * Must be called periodically in {@link frc.robot.subsystems.vision.apriltag.RealATVision#periodic()}
     */
    public void periodic() {
        lastHb = hb;
        hb = LimelightHelpers.getHeartbeat(limelightID);

        if(((int)Timer.getFPGATimestamp()) % 3.0 == 0 && !isConnected()) {
            DriverStation.reportError(limelightID + " is not connected.", false);
        }

        updateTelemetry();
    }

    /**
     * Updates the {@link edu.wpi.first.networktables.NetworkTable NetworkTable} subtable for the Limelight.
     * Records the latest pose estimate, whether or not the Limelight has estimate data, the current
     * {@link limelight.networktables.LimelightPoseEstimator.EstimationMode EstimationMode} for the robot, and the default
     * {@link limelight.networktables.LimelightPoseEstimator.EstimationMode EstimationMode}.
     */
    private void updateTelemetry() {
        if(true) {
            var poseOpt = getPose();
            estimatePublisher.accept(poseOpt.isPresent() ? new Pose3d(poseOpt.get().estimatedPose) : Pose3d.kZero);
            isActivePublisher.accept(hasTargets());
            modePublisher.accept(lastMode.toString());
            defaultModePublisher.accept(defaultMode.toString());
        }
        isConnectedPublisher.accept(isConnected());
    }

    public Pose3d getOffset() {
        return LimelightHelpers.getCameraPose3d_RobotSpace(limelightID);
    }

    public void updateOffset(Pose3d offset) {
        Rotation3d cameraRot = offset.getRotation();
        LimelightHelpers.setCameraPose_RobotSpace(
            limelightID,
            offset.getX(),
            offset.getY(),
            offset.getZ(),
            Math.toDegrees(cameraRot.getX()),
            Math.toDegrees(cameraRot.getY()),
            Math.toDegrees(cameraRot.getZ())
        );
    }

    /**
     * Checks if the heartbeat value of the limelight has updated
     * 
     * @return Whether or not the Limelight is connected
     */
    public boolean isConnected() {
        return hb != lastHb;
    }

    public boolean hasTargets() {
        return LimelightHelpers.getTV(limelightID);
    }
    

    /**
     * Retrieves the pose of the robot. Automatically swaps between MegaTag1 and MegaTag2 depending on the  
     * {@link AprilTagModule#defaultMode}. Returns {@link java.util.Optional#empty()}
     * if there are no results.
     * 
     * @return The estimated pose if the Limelight has targets
     */
    public Optional<AprilTagEstimate> getPose() {
        return getPose(defaultMode == EstimationMode.MEGATAG2);
    }

    // modified version of LL Helpers getPose method
    public Optional<AprilTagEstimate> getPose(boolean isMegaTag2) {
        DoubleArrayEntry poseEntry = LimelightHelpers.getLimelightDoubleArrayEntry(limelightID, isMegaTag2 ? "botpose_orb" : "botpose");
        
        TimestampedDoubleArray tsValue = poseEntry.getAtomic();
        double[] poseArray = tsValue.value;
        long timestamp = tsValue.timestamp;
        
        if (poseArray.length == 0) {
            // Handle the case where no data is available
            return Optional.empty();
        }
    
        var pose = LimelightHelpers.toPose2D(poseArray);
        double latency = LimelightHelpers.extractArrayEntry(poseArray, 6);
        int tagCount = (int) LimelightHelpers.extractArrayEntry(poseArray, 7);
        // double tagSpan = LimelightHelpers.extractArrayEntry(poseArray, 8);
        double tagDist = LimelightHelpers.extractArrayEntry(poseArray, 9);
        // double tagArea = LimelightHelpers.extractArrayEntry(poseArray, 10);
        
        // Convert server timestamp from microseconds to seconds and adjust for latency
        double adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);
    
        double avgAmbiguity = 0;

        int valsPerFiducial = 7;
        int expectedTotalVals = 11 + valsPerFiducial * tagCount;

        if (poseArray.length != expectedTotalVals) {
            // Array size mismatch - return empty array instead of null-filled array
            return Optional.empty();
        } else {
            for(int i = 0; i < tagCount; i++) {
                int baseIndex = 11 + (i * valsPerFiducial);
                avgAmbiguity += poseArray[baseIndex + 6];
            }
        }

        avgAmbiguity /= tagCount;

        lastMode = isMegaTag2 ? EstimationMode.MEGATAG2 : EstimationMode.MEGATAG1;
        return Optional.of(new AprilTagEstimate(pose, adjustedTimestamp, isMegaTag2, tagDist, tagCount, avgAmbiguity));
    }

    /**
     * Seeds the initial orientation of the Limelight for MegaTag2.
     * 
     * @param orientation3d The rotation and angular velocity of the robot.
     */
    public void seedOrientation(Rotation3d rot) {
        LimelightHelpers.SetRobotOrientation(
            limelightID,
            rot.getZ(),
            0,
            rot.getY(),
            0,
            rot.getX(),
            0
        );
    }

    public void captureRewind(double seconds) {
        LimelightHelpers.triggerRewindCapture(limelightID, seconds);
    }
}
