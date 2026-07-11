package frc.robot.subsystems.vision.apriltag;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation3d;
import limelight.networktables.PoseEstimate;

public class NoneATVision extends AprilTagVision {
    public NoneATVision() {}
    public NoneATVision(Supplier<Rotation3d> gyroRotation, Consumer<Rotation3d> resetRotation) {}
    
    @Override
    public void periodic() {}

    @Override
    public void captureRewinds(double seconds) {}

    @Override
    public List<PoseEstimate> getAllEstimates() {
        return estimates;
    }   
}
