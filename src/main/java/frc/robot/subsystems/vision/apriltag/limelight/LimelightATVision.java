package frc.robot.subsystems.vision.apriltag.limelight;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.vision.apriltag.limelight.LimelightATModule.AprilTagEstimate;
import frc.robot.subsystems.vision.apriltag.limelight.LimelightATModule.LimelightMode;

public abstract class LimelightATVision {
    public static class LimelightATVisionConstants {
        public static final String[] LL_IDS = {
            // "limelight-turret"
        };

        public static final Pose3d[] LL_OFFSETS = {
            // new Pose3d( // climb
            //     new Translation3d(Inches.of(-11.0672),Inches.of(-10.432), Inches.of(8.674)),
            //     new Rotation3d(Degrees.zero(), Degrees.of(22.5), Degrees.of(180))),
            // new Pose3d( // right
            //     new Translation3d(Inches.of(2.550), Inches.of(12.987),Inches.of(7.435)),
            //     new Rotation3d(Degrees.zero(), Degrees.of(22.5), Degrees.of(-90))),
            // new Pose3d( // left
            //     new Translation3d(Inches.of(2.550), Inches.of(-12.987), Inches.of(7.435)),
            //     new Rotation3d(Degrees.zero(), Degrees.of(22.5), Degrees.of(90))),
            // new Pose3d( // front
            //     new Translation3d(Inches.of(-11.213), Inches.of(7.375), Inches.of(20.849)),
            //     new Rotation3d(Degrees.zero(), Degrees.of(30), Degrees.zero())
            // )
        };
        public static final LimelightMode kDefaultMode = LimelightMode.MEGATAG1;

        public static final double[] kMT2StdDevCoefficients = {0.085, 0.0}; // deviation order is [xy, theta]
        public static final double[] kMT1StdDevCoefficients = {0.1, 0.075};
        public static final int kOptimalTagCount = 2;
    }
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

    public static Matrix<N3, N1> getStdDevs(AprilTagEstimate estimate) {
        return estimate.isMegaTag2() ? getStdDevsMT2(estimate) : getStdDevsMT1(estimate);
    }

    public static Matrix<N3, N1> getDisabledStdDevs(AprilTagEstimate estimate) {
        return VecBuilder.fill(
                0.01,
                0.01,
                0.01);
    }

    private static Matrix<N3, N1> getStdDevsMT2(AprilTagEstimate estimate) {
        double xydevs = LimelightATVisionConstants.kMT2StdDevCoefficients[0] / estimate.tagArea() / LimelightATVisionConstants.kOptimalTagCount;
        return VecBuilder.fill(
                xydevs,
                xydevs,
                Double.POSITIVE_INFINITY);
    }

    private static Matrix<N3, N1> getStdDevsMT1(AprilTagEstimate estimate) {
        double xydevs = LimelightATVisionConstants.kMT1StdDevCoefficients[0] / estimate.tagArea() / LimelightATVisionConstants.kOptimalTagCount;
        double thetadevs = LimelightATVisionConstants.kMT1StdDevCoefficients[1] / estimate.tagArea() / LimelightATVisionConstants.kOptimalTagCount;
        return VecBuilder.fill(
                xydevs,
                xydevs,
                thetadevs);
    }
}
