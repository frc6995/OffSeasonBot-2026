package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import static frc.robot.util.AllianceFlipUtil.flipped;

public class POI {
    // ============= POSES =============
    public static final Supplier<Pose2d> HUB_CENTER = flipped(
            new Pose2d(4.624246120452881, 4.037848949432373, Rotation2d.kZero));

    // ============= ZONES =============
    // Blue-alliance-relative corners of the zone where a shot should PASS
    // instead of SCORE.
    private static final Translation2d PASSING_ZONE_CORNER_A = new Translation2d(5.0, -2.0);
    private static final Translation2d PASSING_ZONE_CORNER_B = new Translation2d(24.0, 10.0);

    public static final Supplier<Rectangle2d> PASSING_ZONE = flipped(
            new Rectangle2d(PASSING_ZONE_CORNER_A, PASSING_ZONE_CORNER_B));

    public static final Supplier<Rotation2d> PASSING_ANGLE = flipped(Rotation2d.fromDegrees(180));

    // Endpoints of the field-wall line used for the passing shot
    public static final Supplier<Translation2d> PASSING_WALL_START = flipped(new Translation2d(0.0, 0.0));
    public static final Supplier<Translation2d> PASSING_WALL_END = flipped(new Translation2d(0.0, 10.0));

    private POI() {
    }
}
