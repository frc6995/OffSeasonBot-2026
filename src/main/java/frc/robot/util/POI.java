package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.generated.ChoreoVars;
import choreo.Choreo;
import choreo.util.ChoreoAllianceFlipUtil;
import static frc.robot.util.AllianceFlipUtil.flipped;

public class POI {
    // ============= POSES =============
    // Need to flip
    public static final Supplier<Pose2d> HUB_CENTER = flipped(
            new Pose2d(4.624246120452881, 4.037848949432373, Rotation2d.kZero));

    public static final Supplier<Pose2d> TRENCH_START = flipped(ChoreoVars.Poses.TRENCH_START);
    public static final Supplier<Pose2d> M_1 = flipped(ChoreoVars.Poses.M_1);
    public static final Supplier<Pose2d> M_2 = flipped(ChoreoVars.Poses.M_2);
    public static final Supplier<Pose2d> M_3 = flipped(ChoreoVars.Poses.M_3);
    public static final Supplier<Pose2d> HUB_BEHIND_INTAKE = flipped(ChoreoVars.Poses.HUB_BEHIND_INTAKE);

    // ============= ZONES =============
    // Blue-alliance-relative corners of the zone where a shot should PASS
    // instead of SCORE.
    private static final Translation2d PASSING_ZONE_CORNER_A = new Translation2d(5.0, -2.0);
    private static final Translation2d PASSING_ZONE_CORNER_B = new Translation2d(24.0, 10.0);

    public static final Supplier<Rectangle2d> PASSING_ZONE = flipped(
            new Rectangle2d(PASSING_ZONE_CORNER_A, PASSING_ZONE_CORNER_B));

    private POI() {
    }
}
