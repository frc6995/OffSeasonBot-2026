package frc.robot.subsystems.vision.apriltag;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.apriltag.RealATVision.ATVisionConstants;
import limelight.Limelight;
import limelight.networktables.Orientation3d;
import limelight.networktables.LimelightSettings.ImuMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.PoseEstimate;
import limelight.networktables.LimelightPoseEstimator.BotPose;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;

/**
 * Wrapper class for a Yet Another Limelight Library {@link limelight.Limelight} object. 
 * Records vision data to NetworkTables for debugging. 
 */
public class AprilTagModule {
    private final LimelightHelpers limelight;

    private final NetworkTable moduleSubTable;

    private final StructPublisher<Pose3d> robotToCameraPublisher;
    private final StructPublisher<Pose3d> estimatePublisher;
    private final BooleanPublisher isActivePublisher;
    private final StringPublisher modePublisher;
    private final StringPublisher defaultModePublisher;

    public final DoubleArrayPublisher rewindPublisher;
    public final DoubleArraySubscriber rewindSubscriber;
    private PoseEstimate mt1Pose;
    private PoseEstimate mt2Pose;

    private EstimationMode defaultMode;
    private EstimationMode lastMode;

    public AprilTagModule(String limelightID, Pose3d offset, NetworkTable visionTable) {
        this.limelight = new Limelight(limelightID);
        limelight.getSettings()
            .withLimelightLEDMode(LEDMode.PipelineControl)
            .withCameraOffset(offset)
            .withImuMode(ImuMode.ExternalImu)
            // .withImuAssistAlpha()
            .save();

        defaultMode = ATVisionConstants.kDefaultMode;

            // Publishers for Limelight data
        moduleSubTable = visionTable.getSubTable(limelightID);
        robotToCameraPublisher = moduleSubTable.getStructTopic("CameraOffset", Pose3d.struct).publish();
        estimatePublisher = moduleSubTable.getStructTopic("PoseEstimate", Pose3d.struct).publish();
        isActivePublisher = moduleSubTable.getBooleanTopic("IsActive").publish();
        modePublisher = moduleSubTable.getStringTopic("LastEstimateMode").publish();
        defaultModePublisher = moduleSubTable.getStringTopic("DefaultEstimateMode").publish();

        rewindPublisher = moduleSubTable.getDoubleArrayTopic("capture_rewind").publish();
        rewindSubscriber = moduleSubTable.getDoubleArrayTopic("capture_rewind").subscribe(new double[] {0,0});
        
        mt1Pose = new PoseEstimate(limelight, "botpose_wpiblue", false);
        mt2Pose = new PoseEstimate(limelight, "botpose_orb_wpiblue", true);

        // setDouble(1) to enable rewind, 0 to disable
        moduleSubTable.getEntry("rewind_enable_set").setDouble(1);
        robotToCameraPublisher.accept(offset);
        defaultModePublisher.setDefault(defaultMode.name());
    }

    /**
     * Must be called periodically in {@link frc.robot.subsystems.vision.apriltag.RealATVision#periodic()}
     */
    public void periodic() {
        updateTelemetry();
    }

    /**
     * Updates the {@link edu.wpi.first.networktables.NetworkTable} subtable for the Limelight.
     * Records the latest pose estimate, whether or not the Limelight has estimate data, the current
     * {@link limelight.networktables.LimelightPoseEstimator.EstimationMode} for the robot, and the default
     * {@link limelight.networktables.LimelightPoseEstimator.EstimationMode}.
     */
    private void updateTelemetry() {
        if(RobotContainer.kTelemetryVerbosity.compareTo(TelemetryVerbosity.MID) >= 0) {
            var poseSupp = getPose();
            estimatePublisher.accept(poseSupp.isPresent() ? poseSupp.get().pose : Pose3d.kZero);
            isActivePublisher.accept(isActive());
            modePublisher.accept(lastMode.toString());
            defaultModePublisher.accept(defaultMode.toString());
        }
    }

    /**
     * Checks if there is an AprilTag pose estimation.
     * 
     * @return Whether or not the Limelight is estimating a robot pose.
     */
    public boolean isActive() {
        var posemt1 = getPoseMT1();
        return posemt1.isPresent() && posemt1.get().hasData;
    }

    /**
     * Retrieves the pose of the robot. Automatically swaps between MegaTag1 and MegaTag2 depending on the  
     * {@link AprilTagModule#defaultMode}. Returns {@link java.util.Optional#empty()}
     * if there are no results.
     * 
     * @return The estimated pose if the Limelight has targets
     */
    public Optional<PoseEstimate> getPose() {
        if(!isActive()) return Optional.empty();

        switch(defaultMode) {
            case MEGATAG1:
                return getPoseMT1();
            default:
                return getPoseMT2();
        }
    }

    /**
     * Retrieves the pose of the robot using MegaTag2. Must be seeded with an initial orientation before
     * use.
     * 
     * @return The estimated pose if the Limelight has targets and the initial orientation is seeded.
     */
    public Optional<PoseEstimate> getPoseMT2() {
        lastMode = EstimationMode.MEGATAG2;
        return mt2Pose.getPoseEstimate();
    }

    /**
     * Retrieves the pose of the robot using MegaTag1.
     * 
     * @return The estimated pose if the Limelight has targets
     */
    public Optional<PoseEstimate> getPoseMT1() {
        lastMode = EstimationMode.MEGATAG1;
        return mt1Pose.getPoseEstimate();
    }

    /**
     * Seeds the initial orientation of the Limelight for MegaTag2.
     * 
     * @param orientation3d The rotation and angular velocity of the robot.
     */
    public void seedOrientation(Orientation3d orientation3d) {
        limelight.getSettings().withRobotOrientation(orientation3d);
    }
}
