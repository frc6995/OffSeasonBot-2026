package frc.robot.subsystems.vision.apriltag;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.vision.apriltag.AprilTagModule.AprilTagEstimate;

public class NoneATVision extends AprilTagVision {
    public NoneATVision() {}
    public NoneATVision(Supplier<Rotation3d> gyroRotation, Consumer<Rotation3d> resetRotation) {}

    protected void captureRewinds(double seconds) {}
    
    @Override
    public void periodic() {}

    @Override
    public List<AprilTagEstimate> getAllEstimates() {
        return estimates;
    }
    @Override
    public void updateOffsets(Pose3d[] offsets) {}
    
}
