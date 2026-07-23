package frc.robot.subsystems.vision.apriltag;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.vision.apriltag.AprilTagModule.EstimationMode;

public class RealATVision extends AprilTagVision {
    public static class ATVisionConstants {
        public static final String[] LL_IDS = {
            "limelight-climb", 
            "limelight-right", 
            "limelight-left",
            "limelight-front"
        };


        public static final Pose3d[] LL_OFFSETS = {
            new Pose3d( // climb
                new Translation3d(Inches.of(-11.0672),Inches.of(-10.432), Inches.of(8.674)),
                new Rotation3d(Degrees.zero(), Degrees.of(22.5), Degrees.of(180))),
            new Pose3d( // right
                new Translation3d(Inches.of(2.550), Inches.of(12.987),Inches.of(7.435)),
                new Rotation3d(Degrees.zero(), Degrees.of(22.5), Degrees.of(-90))),
            new Pose3d( // left
                new Translation3d(Inches.of(2.550), Inches.of(-12.987), Inches.of(7.435)),
                new Rotation3d(Degrees.zero(), Degrees.of(22.5), Degrees.of(90))),
            new Pose3d( // front
                new Translation3d(Inches.of(-11.213), Inches.of(7.375), Inches.of(20.849)),
                new Rotation3d(Degrees.zero(), Degrees.of(30), Degrees.zero())
            )
        };
        public static final EstimationMode kDefaultMode = EstimationMode.MEGATAG1;

        public static final double[] kMT2StdDevCoefficients = {0.085, 0.0}; // deviation order is [xy, theta]
        public static final double[] kMT1StdDevCoefficients = {0.1, 0.075};

    }

    private AprilTagModule[] limelights;

    private final Supplier<Rotation3d> gyroRotation;
    private final Consumer<Pose2d> resetPose;

    private final NetworkTable visionTable;

    private boolean headingSeeded = false;

    private final BooleanPublisher headingSeededPublisher;
    private final StructPublisher<Pose3d> seededPosePublisher;

    public RealATVision(Supplier<Rotation3d> gyroRotation, Consumer<Pose2d> resetPose) {
        this.gyroRotation = gyroRotation;
        this.resetPose = resetPose;

        limelights = new AprilTagModule[ATVisionConstants.LL_IDS.length];

        visionTable = NetworkTableInstance.getDefault().getTable("Vision");
        headingSeededPublisher = visionTable.getBooleanTopic("HeadingSeeded").publish();
        seededPosePublisher = visionTable.getStructTopic("SeededPose", Pose3d.struct).publish();

        for(int i = 0; i < limelights.length; i++) {
            limelights[i] = new AprilTagModule(ATVisionConstants.LL_IDS[i], ATVisionConstants.LL_OFFSETS[i], visionTable);
        }
    }

    public void periodic() {
        estimates.clear();
        if(DriverStation.isDisabled() || !headingSeeded) {
            for(AprilTagModule limelight : limelights) {
                limelight.periodic();
                var result = limelight.getPose(false);
                if(result.isPresent() && result.get().estimatedPose().getTranslation().getDistance(Translation2d.kZero) > 0.05) {
                    estimates.add(result.get());
                    headingSeeded = true;
                }
            }
            seededPosePublisher.accept(new Pose3d(Translation3d.kZero, gyroRotation.get()));
        } else {
            if(!headingSeeded) headingSeeded = true;
                        
            for(AprilTagModule limelight : limelights) {
                limelight.periodic();
                
                limelight.seedOrientation(
                    gyroRotation.get()
                );
                var estSupp = limelight.getPose();

                if(estSupp.isPresent()) {
                    estimates.add(estSupp.get());
                }
            }
        }
        headingSeededPublisher.accept(headingSeeded);
    }

    @Override
    public void updateOffsets(Pose3d[] offsets) {
        if(offsets.length != limelights.length) return;
        for(int i = 0; i < limelights.length; i++) {
            if(offsets[i] == null || ATVisionConstants.LL_OFFSETS[i].equals(limelights[i].getOffset())) continue;
            limelights[i].updateOffset(offsets[i]);
        }
    }

    public Command captureRewindsCommand(double seconds) {
        return Commands.runOnce(() -> captureRewinds(seconds));
    }

    protected void captureRewinds(double seconds) {
        for(AprilTagModule cam : limelights) {
            cam.captureRewind(seconds);
        }
    }
}