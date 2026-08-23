package frc.robot.subsystems.vision.apriltag;

import edu.wpi.first.math.geometry.Pose3d;

public class NoneATVision extends AprilTagVision {
    public NoneATVision() {}

    @Override
    protected void captureRewinds(double seconds) {}

    @Override
    public void periodic() {}

    @Override
    public void updateOffsets(Pose3d[] offsets) {}
}
