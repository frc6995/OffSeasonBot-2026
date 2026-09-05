package frc.robot.subsystems.vision.photon;


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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.vision.ATVision.ATVisionConstants;

import java.util.ArrayList;
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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.util.List;

public class RealPhotonATVision {
    public class PhotonVisionConstants {
        public static final String[] PHOTON_IDS = {
            "photon-right",
            "photon-left"
        };

        public static final Transform3d[] PHOTON_OFFSETS = {
            new Transform3d(
                new Translation3d(Inches.of(0), Inches.of(0), Inches.of(0)),
                new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0))
            ),
            new Transform3d(
                new Translation3d(Inches.of(0), Inches.of(0), Inches.of(0)),
                new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0))
            )
        };
        
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

        public static final double[] kStdDevCoefficients = {0.5, 1};

        public static final String TABLE_ID = "Photon";
    }

    private PhotonATModule[] cameras;

    private final NetworkTable photonTable;

    public RealPhotonATVision(NetworkTable visionTable) {
        cameras = new PhotonATModule[PhotonVisionConstants.PHOTON_IDS.length];

        photonTable = visionTable.getSubTable(PhotonVisionConstants.TABLE_ID);

        for(int i = 0; i < cameras.length; i++) {
            cameras[i] = new PhotonATModule(PhotonVisionConstants.PHOTON_IDS[i], PhotonVisionConstants.PHOTON_OFFSETS[i], photonTable);
        }
    }

    public void periodic() {
        for(var cam : cameras) {
            cam.periodic();
        }
    }

    public ArrayList<EstimatedRobotPose> getLatestEstimates() {
        var estimates = new ArrayList<EstimatedRobotPose>();
        for (var cam : cameras) {
            estimates.addAll(cam.getLatestEstimates());
        }
        return estimates;
    }

    public Matrix<N3, N1> getEstimationStdDevs(List<PhotonTrackedTarget> targets) {
        double tagArea = 0;
        for(var target : targets) tagArea += target.getArea();

        double xydevs = PhotonVisionConstants.kStdDevCoefficients[0] / tagArea / ATVisionConstants.kOptimalTagCount;
        double thetadevs = PhotonVisionConstants.kStdDevCoefficients[1] / tagArea / ATVisionConstants.kOptimalTagCount;
        return VecBuilder.fill(
                xydevs,
                xydevs,
                thetadevs);
    }
}
