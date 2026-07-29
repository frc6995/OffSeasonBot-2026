package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import choreo.Choreo;
import choreo.util.ChoreoAllianceFlipUtil;
import static frc.robot.util.AllianceFlipUtil.flipped;


public class POI {
    // ============= POSES =============
//Need to flip
    public static final Supplier<Pose2d> TEST_POSE = flipped(new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(90.0)));
        
    public static final Supplier<Pose2d> HUB_CENTER = flipped(new Pose2d(4.624246120452881, 4.037848949432373, Rotation2d.kZero));

    private POI() {
    }
}
