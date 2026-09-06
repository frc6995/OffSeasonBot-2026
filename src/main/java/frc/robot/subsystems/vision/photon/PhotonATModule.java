package frc.robot.subsystems.vision.photon;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.subsystems.vision.photon.RealPhotonATVision.PhotonVisionConstants;

public class PhotonATModule {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator estimator;

    private final NetworkTable cameraTable;
    private final StructPublisher<Pose3d> posePublisher;
    private final BooleanPublisher connectedPublisher;

    public PhotonATModule(String cameraID, Transform3d offset, NetworkTable visionTable) {
        camera = new PhotonCamera(cameraID);
        estimator = new PhotonPoseEstimator(PhotonVisionConstants.kTagLayout, offset);

        cameraTable = visionTable.getSubTable(cameraID);
        posePublisher = cameraTable.getStructTopic("Estimate", Pose3d.struct).publish();
        connectedPublisher = cameraTable.getBooleanTopic("Is Connected").publish();
    }

    public void periodic() {
        updateTelemetry();
    }

    public ArrayList<EstimatedRobotPose> getLatestEstimates() {
        var results = camera.getAllUnreadResults();
        var estimates = new ArrayList<EstimatedRobotPose>();
        
        for(var result : results) {
            Optional<EstimatedRobotPose> estimate = estimator.estimateCoprocMultiTagPose(result);
            estimate.ifPresent((e) -> estimates.add(e));
        }

        return estimates;
    }

    private void updateTelemetry() {
        var estimate = estimator.estimateCoprocMultiTagPose(camera.getLatestResult());
        estimate.ifPresent((e) -> posePublisher.accept(e.estimatedPose));

        connectedPublisher.accept(camera.isConnected());
    }
}
