package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class POI {
    // ============= POSES =============
//Need to flip
    public static final Supplier<Pose2d> TEST_POSE = () -> new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));

    private POI() {
    }
}
