package frc.robot.subsystems.vision.apriltag;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.apriltag.AprilTagModule.AprilTagEstimate;
import frc.robot.subsystems.vision.apriltag.RealATVision.ATVisionConstants;

public abstract class AprilTagVision extends SubsystemBase {
    protected ArrayList<AprilTagEstimate> estimates = new ArrayList<AprilTagEstimate>(0);

    public abstract void updateOffsets(Pose3d[] offsets);

    protected abstract void captureRewinds(double seconds);

    public List<AprilTagEstimate> getAllEstimates() {
        return Collections.unmodifiableList(estimates);
    }

    public Command captureRewindsCommand(double seconds) {
        return Commands.runOnce(() -> captureRewinds(seconds));
    }

    public static Matrix<N3, N1> getStdDevs(AprilTagEstimate estimate) {
        return estimate.isMegaTag2() ? getStdDevsMT2(estimate) : getStdDevsMT1(estimate);
    }

    public static Matrix<N3, N1> getDisabledStdDevs(AprilTagEstimate estimate) {
        // While disabled we trust vision almost completely so the estimator can converge before
        // the match starts. MegaTag2's yaw is just the yaw we fed it, so never trust its theta --
        // doing so would feed the estimator's own heading back to itself.
        return VecBuilder.fill(
                0.01,
                0.01,
                estimate.isMegaTag2() ? Double.POSITIVE_INFINITY : 0.01);
    }

    private static Matrix<N3, N1> getStdDevsMT2(AprilTagEstimate estimate) {
        // MegaTag2 derives its yaw from the orientation we supply, so it carries no independent
        // heading information.
        double xydevs = ATVisionConstants.kMT2StdDevCoefficients[0] * stdDevScale(estimate);
        return VecBuilder.fill(
                xydevs,
                xydevs,
                Double.POSITIVE_INFINITY);
    }

    private static Matrix<N3, N1> getStdDevsMT1(AprilTagEstimate estimate) {
        double scale = stdDevScale(estimate);
        return VecBuilder.fill(
                ATVisionConstants.kMT1StdDevCoefficients[0] * scale,
                ATVisionConstants.kMT1StdDevCoefficients[0] * scale,
                ATVisionConstants.kMT1StdDevCoefficients[1] * scale);
    }

    /**
     * Scales the tuned coefficients by how good the observation actually was: error grows roughly
     * with the square of tag distance and shrinks with more tags in the solve. Normalised so that
     * {@link ATVisionConstants#kOptimalTagCount} tags at 1 m gives a scale of 1, which keeps the
     * coefficients meaning what they meant when they were tuned.
     * <p>
     * Returns POSITIVE_INFINITY for a degenerate estimate; WPILib's pose estimator drives the
     * vision gain to zero for an infinite std dev, so such a measurement is ignored rather than
     * producing NaN.
     */
    private static double stdDevScale(AprilTagEstimate estimate) {
        if (estimate.tagCount() <= 0) return Double.POSITIVE_INFINITY;
        // Below a metre the distance term shouldn't start *increasing* trust past the tuning point.
        double dist = Math.max(estimate.avgTagDistMeters(), 1.0);
        return (dist * dist) * ATVisionConstants.kOptimalTagCount / estimate.tagCount();
    }
}
