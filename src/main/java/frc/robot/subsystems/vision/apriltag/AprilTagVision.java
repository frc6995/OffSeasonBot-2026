package frc.robot.subsystems.vision.apriltag;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.vision.apriltag.AprilTagModule.AprilTagEstimate;
import frc.robot.subsystems.vision.apriltag.AprilTagModule.EstimationMode;

public abstract class AprilTagVision {
    protected final ArrayList<AprilTagEstimate> estimates = new ArrayList<>();

    public abstract void updateOffsets(Pose3d[] offsets);

    public abstract void updateTurretCameraOffset(Pose3d offset);

    public abstract void seedOrientations(Rotation3d rotation);

    public abstract void periodic(EstimationMode mode);

    public abstract boolean isTurretCameraEstimate(AprilTagEstimate estimate);

    public abstract void flush();

    public abstract void captureRewinds(double seconds);

    public List<AprilTagEstimate> getAllEstimates() {
        return Collections.unmodifiableList(estimates);
    }
}
