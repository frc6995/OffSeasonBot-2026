package frc.robot.subsystems.vision.apriltag.photonvision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public abstract class PhotonATVision {
    public static class PhotonATVisionConstants {
        public static final String[] PHOTON_IDS = {
        };

        public static final Transform3d[] PHOTON_OFFSETS = {
        };

        public static final double[] kStdDevCoefficients = {0.085, 0.0};
        public static final int kOptimalTagCount = 2;
    }

    public abstract void periodic();

    public abstract List<PhotonPipelineResult> getAllEstimates();

    public static Matrix<N3, N1> getStdDevs(PhotonPipelineResult estimate) {
        double tagArea = 0;
        for(var target : estimate.getTargets()) {
            tagArea += target.area;
        }
        
        double xydevs = PhotonATVisionConstants.kStdDevCoefficients[0] / tagArea / PhotonATVisionConstants.kOptimalTagCount;
        double thetadevs = PhotonATVisionConstants.kStdDevCoefficients[1] / tagArea / PhotonATVisionConstants.kOptimalTagCount;
        return VecBuilder.fill(
                xydevs,
                xydevs,
                thetadevs); 
    }

    public static Matrix<N3, N1> getDisabledStdDevs() {
        return VecBuilder.fill(
                0.01,
                0.01,
                0.01);
    }
}
