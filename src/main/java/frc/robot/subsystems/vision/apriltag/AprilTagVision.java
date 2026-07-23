package frc.robot.subsystems.vision.apriltag;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.vision.apriltag.AprilTagModule.AprilTagEstimate;
import frc.robot.subsystems.vision.apriltag.RealATVision.ATVisionConstants;

public abstract class AprilTagVision {
    protected ArrayList<AprilTagEstimate> estimates = new ArrayList<AprilTagEstimate>(0);

    public abstract void periodic();
    public abstract void updateOffsets(Pose3d[] offsets);
    protected abstract void captureRewinds(double seconds);

    public List<AprilTagEstimate> getAllEstimates() {
        return estimates;
    }

    public Command captureRewindsCommand(double seconds) {
        return Commands.runOnce(() -> captureRewinds(seconds));
    }

    public static Matrix<N3,N1> getStdDevs(AprilTagEstimate estimate) {
        return estimate.isMegaTag2() ? getStdDevsMT2(estimate) : getStdDevsMT1(estimate);
    }

    public static Matrix<N3,N1> getDisabledStdDevs(AprilTagEstimate estimate) {
        return VecBuilder.fill(
            0.1,
            0.1,
            0.1
        );
    }

    private static Matrix<N3,N1> getStdDevsMT2(AprilTagEstimate estimate) {
        double xydevs = ATVisionConstants.kMT2StdDevCoefficients[0] * Math.pow(estimate.avgTagDistMeters(), 2.0) / Math.pow(estimate.tagCount(), 2.0);
        return VecBuilder.fill(
            xydevs,
            xydevs,
            Double.POSITIVE_INFINITY
        );
    }

    private static Matrix<N3,N1> getStdDevsMT1(AprilTagEstimate estimate) {
        double xydevs = ATVisionConstants.kMT1StdDevCoefficients[0] * Math.pow(estimate.avgTagDistMeters(), 2.0) / Math.pow(estimate.tagCount(), 2.0);
        double thetadevs = ATVisionConstants.kMT1StdDevCoefficients[1] * Math.pow(estimate.avgTagDistMeters(), 2.0) / Math.pow(estimate.tagCount(), 2.0);
        return VecBuilder.fill(
            xydevs,
            xydevs,
            thetadevs
        );
    }
}
