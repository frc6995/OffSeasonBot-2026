package frc.robot.subsystems.vision.apriltag;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.vision.apriltag.RealATVision.ATVisionConstants;
import limelight.networktables.PoseEstimate;

public abstract class AprilTagVision {
    protected ArrayList<PoseEstimate> estimates = new ArrayList<PoseEstimate>(0);

    public abstract void periodic();
    public abstract List<PoseEstimate> getAllEstimates();
    public abstract void captureRewinds(double seconds);

    public static Matrix<N3,N1> getStdDevs(PoseEstimate estimate) {
        double xydevs = ATVisionConstants.kStdDevCoefficients[0] * Math.pow(estimate.avgTagDist, 2.0) / Math.pow(estimate.tagCount, 2.0);
        double thetadevs = estimate.isMegaTag2 ? Double.POSITIVE_INFINITY : ATVisionConstants.kStdDevCoefficients[1] * Math.pow(estimate.avgTagDist, 2.0) / Math.pow(estimate.tagCount, 2.0);
        return VecBuilder.fill(
            xydevs,
            xydevs,
            thetadevs
        );
    }

    public static Matrix<N3,N1> getDisabledStdDevs(PoseEstimate estimate) {
        double xydevs = ATVisionConstants.kDisabledStdDevCoefficients[0] * Math.pow(estimate.avgTagDist, 2.0) / Math.pow(estimate.tagCount, 2.0);
        double thetadevs = ATVisionConstants.kDisabledStdDevCoefficients[1] * Math.pow(estimate.avgTagDist, 2.0) / Math.pow(estimate.tagCount, 2.0);
        return VecBuilder.fill(
            xydevs,
            xydevs,
            thetadevs
        );
    }
}
