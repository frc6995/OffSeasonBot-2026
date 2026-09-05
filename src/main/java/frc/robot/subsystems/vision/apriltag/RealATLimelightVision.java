package frc.robot.subsystems.vision.apriltag;

import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.subsystems.vision.ATVision;
import frc.robot.subsystems.vision.apriltag.AprilTagModule.AprilTagEstimate;
import frc.robot.subsystems.vision.apriltag.AprilTagModule.EstimationMode;
import frc.robot.util.LimelightHelpers;

public class RealATLimelightVision extends AprilTagVision {
    public static class LimelightConstants {
        public static final String[] LL_IDS = {
            "limelight-turret"
        };

        public static final Pose3d[] LL_OFFSETS = {
            ATVision.ATVisionConstants.kInitialTurretCameraOffset
        };

        /** Index into {@link #LL_IDS} of the turret-mounted camera. */
        public static final int kTurretCameraIndex = 0;

        public static final EstimationMode kDefaultMode = EstimationMode.MEGATAG1;

        /**
         * Whether the Limelight's camerapose_robotspace frame is +Y right (making it mirrored
         * relative to WPILib's +Y left), which also flips the sign of roll and yaw.
         */
        public static final boolean kLimelightRobotSpaceIsYRight = false;

        /** How long the heartbeat may stall before the camera counts as disconnected. */
        public static final double kHeartbeatTimeoutSeconds = 0.25;
        /** Minimum spacing between repeated "not connected" driver station errors. */
        public static final double kDisconnectReportPeriodSeconds = 3.0;
    }

    private final AprilTagModule[] limelights;
    /** The turret-mounted camera, or null if no cameras are configured. */
    private final AprilTagModule turretCamera;
    private final Set<AprilTagEstimate> turretEstimates =
        Collections.newSetFromMap(new IdentityHashMap<>());

    public RealATLimelightVision(NetworkTable visionTable) {
        limelights = new AprilTagModule[LimelightConstants.LL_IDS.length];

        for (int i = 0; i < limelights.length; i++) {
            limelights[i] = new AprilTagModule(
                LimelightConstants.LL_IDS[i],
                LimelightConstants.LL_OFFSETS[i],
                visionTable);
        }

        turretCamera = (LimelightConstants.kTurretCameraIndex < limelights.length)
            ? limelights[LimelightConstants.kTurretCameraIndex]
            : null;
    }

    @Override
    public void seedOrientations(Rotation3d rotation) {
        for (AprilTagModule limelight : limelights) {
            limelight.seedOrientation(rotation);
        }
    }

    @Override
    public void periodic(EstimationMode mode) {
        estimates.clear();
        turretEstimates.clear();

        for (int i = 0; i < limelights.length; i++) {
            AprilTagModule limelight = limelights[i];
            limelight.periodic(mode);

            var estOpt = limelight.getLatestEstimate();
            if (estOpt.isEmpty()) continue;

            AprilTagEstimate estimate = estOpt.get();
            estimates.add(estimate);
            if (i == LimelightConstants.kTurretCameraIndex) {
                turretEstimates.add(estimate);
            }
        }
    }

    @Override
    public boolean isTurretCameraEstimate(AprilTagEstimate estimate) {
        return turretEstimates.contains(estimate);
    }

    @Override
    public void updateTurretCameraOffset(Pose3d offset) {
        if (turretCamera != null) {
            turretCamera.updateOffset(offset);
        }
    }

    @Override
    public void updateOffsets(Pose3d[] offsets) {
        if (offsets.length != limelights.length) return;
        for (int i = 0; i < limelights.length; i++) {
            if (offsets[i] == null || i == LimelightConstants.kTurretCameraIndex) continue;
            limelights[i].updateOffset(offsets[i]);
        }
    }

    @Override
    public void flush() {
        if (limelights.length > 0) {
            LimelightHelpers.Flush();
        }
    }

    @Override
    public void captureRewinds(double seconds) {
        for (AprilTagModule cam : limelights) {
            cam.captureRewind(seconds);
        }
    }
}
