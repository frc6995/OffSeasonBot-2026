package frc.robot.subsystems.vision;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class RealATPhotonVision {
    public class PhotonVisionConstants {

        //peyton didn't let me name them "peyton", and "matthew"
        public static final String rightPhotonCameraName = "rightCam";
        public static final String leftPhotonCameraName = "leftCam";
        
        private static final double rightPhotonCamRoll = Units.degreesToRadians(30); //TODO: for testing, check what the camera pitch is
        private static final double leftPhotonCamRoll = Units.degreesToRadians(-30); //TODO do ts

        private static final double rightPhotonCamYaw = Units.degreesToRadians(-90);
        private static final double leftPhotonCamYaw = Units.degreesToRadians(90);
        // See https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#robot-coordinate-system
        // for why these values the way they are. In short x is positive towards the front, y is positive to left, z is positive to the sky
        //set these lolxd
        public static final Transform3d robotToRightPhotonCam =
                new Transform3d(new Translation3d(Units.inchesToMeters(-9.37346), Units.inchesToMeters(-9.58278), Units.inchesToMeters(-21.27992)), new Rotation3d(rightPhotonCamRoll, 0, rightPhotonCamYaw));
        
        public static final Transform3d robotToleftPhotonCam =
                new Transform3d(new Translation3d(Units.inchesToMeters(-12.00616), Units.inchesToMeters(-9.58278), Units.inchesToMeters(-21.27992)), new Rotation3d(leftPhotonCamRoll, 0, leftPhotonCamYaw)); 

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // The standard deviations of our vision estimated poses, which affect correction rate
        // TODO: (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

}
    private final PhotonCamera rightPhotonCamera;
    private final PhotonCamera leftPhotonCamera;

    private PhotonCamera[] cameras;

    private final PhotonPoseEstimator rightPhotonPoseEstimator;
    private final PhotonPoseEstimator leftPhotonPoseEstimator;
    private Matrix<N3, N1> curStdDevs = PhotonVisionConstants.kSingleTagStdDevs;

    private final StructPublisher<Pose2d> rightPhotonPoseEstimatorPublisher;
    private final StructPublisher<Pose2d> leftPhotonPoseEstimatorPublisher;

    // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    public RealATPhotonVision() {
        rightPhotonCamera = new PhotonCamera(PhotonVisionConstants.rightPhotonCameraName);
        leftPhotonCamera = new PhotonCamera(PhotonVisionConstants.leftPhotonCameraName);
        
        cameras = new PhotonCamera[] {rightPhotonCamera, leftPhotonCamera};

        // Create PhotonPoseEstimator with MULTI_TAG_PNP_ON_COPROCESSOR as primary strategy
        rightPhotonPoseEstimator = new PhotonPoseEstimator(
            PhotonVisionConstants.kTagLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            PhotonVisionConstants.robotToRightPhotonCam
        );
        leftPhotonPoseEstimator = new PhotonPoseEstimator(
           PhotonVisionConstants.kTagLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            PhotonVisionConstants.robotToleftPhotonCam
        );
    
        // Set fallback strategy
        rightPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        leftPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        rightPhotonPoseEstimatorPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("/PoseEstimator/rightPhotonPose", Pose2d.struct).publish();

        leftPhotonPoseEstimatorPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("/PoseEstimator/leftPhotonPose", Pose2d.struct).publish();

        // ----- Simulation ------
        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim.addAprilTags(PhotonVisionConstants.kTagLayout);
            TargetModel targetModel = new TargetModel(0.5,0.25);
            Pose3d targetPose = new Pose3d(16,4,2, new Rotation3d(0,0,Math.PI));

            VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);
            // Create simulated camera properties. These can be set to mimic your actual camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.
            cameraSim = new PhotonCameraSim(rightPhotonCamera, cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(cameraSim, PhotonVisionConstants.robotToRightPhotonCam);

            cameraSim.enableDrawWireframe(true);
        }
    }

        /**
     * Get the estimated robot pose from the rightPhoton camera.
     * This method processes all unread results and returns the most recent estimate.
     * 
     * @return Optional containing the estimated robot pose, or empty if no valid estimate
     */
    public Optional<EstimatedRobotPose> getrightPhotonEstimatedGlobalPose() {
        
        // var result = rightPhotonCamera.getLatestResult();
        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        for (var result : rightPhotonCamera.getAllUnreadResults()) {
            // // Only process if we have targets
            // if (!result.hasTargets()) {
            // curStdDevs = kSingleTagStdDevs;
            // return Optional.empty();
            // }
            
            // Update the pose estimator with the latest result
            visionEst = rightPhotonPoseEstimator.estimateCoprocMultiTagPose(result);
        
            // Update standard deviations based on the estimate quality
            updateEstimationStdDevs(visionEst, result.getTargets());

            if (visionEst.isPresent()) {
            rightPhotonPoseEstimatorPublisher.set(visionEst.get().estimatedPose.toPose2d());
            }
            
        }
        

        
        // // Debug output
        // if (visionEst.isPresent()) {
        //     SmartDashboard.putString("Vision/EstimatedPose", 
        //         String.format("(%.2f, %.2f, %.2f°)", 
        //             visionEst.get().estimatedPose.getX(),
        //             visionEst.get().estimatedPose.getY(),
        //             visionEst.get().estimatedPose.getRotation().toRotation2d().getDegrees()));
        //     SmartDashboard.putNumber("Vision/NumTargets", result.getTargets().size());
        //     SmartDashboard.putNumber("Vision/Timestamp", visionEst.get().timestampSeconds);
        // } else {
        //     SmartDashboard.putString("Vision/EstimatedPose", "No Estimate");
        // }
        
        return visionEst;
    }
        /**
     * Get the estimated robot pose from the leftPhoton camera.
     * This method processes all unread results and returns the most recent estimate.
     * 
     * @return Optional containing the estimated robot pose, or empty if no valid estimate
     */
    public Optional<EstimatedRobotPose> getleftPhotonEstimatedGlobalPose() {
        
        // var result = rightPhotonCamera.getLatestResult();
        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        for (var result : leftPhotonCamera.getAllUnreadResults()) {
            // // Only process if we have targets
            // if (!result.hasTargets()) {
            // curStdDevs = kSingleTagStdDevs;
            // return Optional.empty();
            // }

            // Update the pose estimator with the latest result

            leftPhotonPoseEstimator.estimateCoprocMultiTagPose(result);
        
            // Update standard deviations based on the estimate quality
            updateEstimationStdDevs(visionEst, result.getTargets());

            if (visionEst.isPresent()) {
            leftPhotonPoseEstimatorPublisher.set(visionEst.get().estimatedPose.toPose2d());
            }
        }
        

        
        // // Debug output
        // if (visionEst.isPresent()) {
        //     SmartDashboard.putString("Vision/EstimatedPose", 
        //         String.format("(%.2f, %.2f, %.2f°)", 
        //             visionEst.get().estimatedPose.getX(),
        //             visionEst.get().estimatedPose.getY(),
        //             visionEst.get().estimatedPose.getRotation().toRotation2d().getDegrees()));
        //     SmartDashboard.putNumber("Vision/NumTargets", result.getTargets().size());
        //     SmartDashboard.putNumber("Vision/Timestamp", visionEst.get().timestampSeconds);
        // } else {
        //     SmartDashboard.putString("Vision/EstimatedPose", "No Estimate");
        // }
        
        return visionEst;
    }
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, 
            List<PhotonTrackedTarget> targets) {
        
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = PhotonVisionConstants.kSingleTagStdDevs;
            SmartDashboard.putString("Vision/StdDevs", "Single Tag Default");
            return;
        }

        // Pose present. Start running Heuristic
        var estStdDevs = PhotonVisionConstants.kSingleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        // Precalculation - see how many tags we found, and calculate an average-distance metric
        for (var tgt : targets) {
            var tagPose = rightPhotonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation()
                    .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) {
            // No valid tags visible. Default to single-tag std devs
            curStdDevs = PhotonVisionConstants.kSingleTagStdDevs;
            SmartDashboard.putString("Vision/StdDevs", "No Valid Tags");
        } else {
            // One or more tags visible, run the full heuristic.
            avgDist /= numTags;
            
            // Decrease std devs if multiple targets are visible
            if (numTags > 1) {
                estStdDevs = PhotonVisionConstants.kMultiTagStdDevs;
            }
            
            // Increase std devs based on (average) distance
            // If single tag and far away, reject the measurement
            if (numTags == 1 && avgDist > 2.5) {
                estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                SmartDashboard.putString("Vision/StdDevs", "Rejected - Too Far");
            } else {
                // Scale std devs based on distance
                estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                SmartDashboard.putString("Vision/StdDevs", 
                    String.format("%d tags, %.2fm avg dist", numTags, avgDist));
            }
            
            curStdDevs = estStdDevs;
        }
        
        // Output the actual std dev values for debugging
        SmartDashboard.putNumber("Vision/StdDev_X", curStdDevs.get(0, 0));
        SmartDashboard.putNumber("Vision/StdDev_Y", curStdDevs.get(1, 0));
        SmartDashboard.putNumber("Vision/StdDev_Theta", curStdDevs.get(2, 0));
    }

    /**
     * Returns the latest standard deviations of the estimated pose, for use with
     * SwerveDrivePoseEstimator. This should only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }
    /**
     * Get location of hub apriltag relative to the shooter camera 
     * @return Optional containing a transform3d of the apriltag on the hub
     */

    // ----- Simulation -----

    public void simulationPeriodic(Pose2d robotSimPose) {
        if (Robot.isSimulation() && visionSim != null) {
            visionSim.update(robotSimPose);
        }
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation() && visionSim != null) {
            visionSim.resetRobotPose(pose);
        }
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation() || visionSim == null) return null;
        return visionSim.getDebugField();
    }
}