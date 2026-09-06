package frc.robot.subsystems.vision.apriltag;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.vision.apriltag.AprilTagModule.AprilTagEstimate;
import frc.robot.subsystems.vision.apriltag.AprilTagModule.EstimationMode;

public class NoneATLimelightVision extends AprilTagVision {
    public NoneATLimelightVision() {}

    @Override
    public void captureRewinds(double seconds) {}

    @Override
    public void periodic(EstimationMode mode) {}

    @Override
    public void updateOffsets(Pose3d[] offsets) {}

    @Override
    public void updateTurretCameraOffset(Pose3d offset) {}

    @Override
    public void seedOrientations(Rotation3d rotation) {}

    @Override
    public boolean isTurretCameraEstimate(AprilTagEstimate estimate) {
        return false;
    }

    @Override
    public void flush() {}
}
