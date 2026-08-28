package frc.robot.subsystems.vision.apriltag;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.apriltag.AprilTagModule.AprilTagEstimate;
import frc.robot.util.LimelightHelpers;

/**
 * Drives {@link RealATVision} in simulation by faking the Limelights' NetworkTables output
 * instead of reading a real camera. Everything downstream of the NT boundary -- {@link
 * AprilTagModule}'s parsing, the heartbeat/connection check, and every accept/reject rule in
 * {@link RealATVision} -- runs completely unmodified against this synthetic data, so this is a
 * test of that pipeline, not a replacement for it.
 * <p>
 * Each loop, before deferring to {@link RealATVision#periodic()}, this:
 * <ul>
 *   <li>projects the field's AprilTags into a simulated camera frame (built from the simulated
 *       drivetrain pose and the live turret angle) to decide which tags are "visible" this frame,
 *   <li>writes noisy {@code botpose_wpiblue} / {@code botpose_orb_wpiblue} arrays and a
 *       monotonic {@code hb} in the exact format {@link AprilTagModule} parses, and
 *   <li>projects the turret camera's tags using the turret angle from
 *       {@link RealATVision.ATVisionConstants#kOffsetTransportLatencySeconds} ago (plus any
 *       injected extra lag), the same way a real Limelight lags behind a live camerapose_robotspace
 *       write -- so {@link RealATVision}'s turret-mismatch rejection is exercised honestly rather
 *       than trivially passing every frame.
 * </ul>
 * <p>
 * <b>Caveat:</b> there is no independent ground-truth pose in this simulator -- {@code
 * swerveState}'s pose is the same fused pose {@link RealATVision} feeds vision measurements back
 * into, so this is a closed loop, not a comparison against physics truth. It's still meaningful
 * for what it's built to test: whether bad estimates (stale, ambiguous, mid-slew, disconnected)
 * get rejected, and whether good ones don't. It says little about absolute pose accuracy.
 */
public class SimATVision extends RealATVision {
    public static class SimVisionConstants {
        /** Approximate Limelight FOV. Adjust to match the hardware actually mounted. */
        public static final double kHorizontalFOVDeg = 82.0;
        public static final double kVerticalFOVDeg = 56.3;

        /** Tags farther than this are treated as not detected, regardless of angle. */
        public static final double kMaxTagRangeMeters = 6.0;

        /** Baked into the botpose array's latency field, so downstream timestamp math is exercised. */
        public static final double kSimLatencyMs = 20.0;

        /** MegaTag1 xy/yaw noise at 1 m with 1 tag; scaled by distance and tag count below. */
        public static final double kMT1BaseXYNoiseMeters = 0.02;
        public static final double kMT1BaseYawNoiseRad = Math.toRadians(1.0);
        /** MegaTag2 only solves translation -- yaw is echoed back from what we seeded it with. */
        public static final double kMT2XYNoiseMeters = 0.03;

        /**
         * Ambiguity model: a base value, plus terms that grow with how edge-on (as opposed to
         * square-on) the tag is viewed and with range, both divided by the tag count for that
         * frame since a multi-tag solve is far better constrained than any single tag alone.
         */
        public static final double kBaseAmbiguity = 0.02;
        public static final double kAmbiguityAngleCoeff = 0.4;
        public static final double kAmbiguityDistCoeff = 0.3;
        public static final double kAmbiguityNoiseStdDev = 0.02;

        /** Fake tag area (percent of image) at 1 m range; falls off with range^2. */
        public static final double kTagAreaAtOneMeterPct = 8.0;
    }

    private record SimObservedTag(int id, double rangeMeters, double areaPercent, double ambiguity) {}

    private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    private final Supplier<SwerveDriveState> swerveState;
    private final Supplier<Rotation3d> gyroRotation;
    private final Supplier<Double> turretAngleSupplier;

    /** Shadows {@link RealATVision}'s own pushed-angle history so the turret-mismatch geometry can be reproduced here. */
    private final TimeInterpolatableBuffer<Rotation2d> pushedTurretAngleHistory =
        TimeInterpolatableBuffer.createBuffer(ATVisionConstants.kEstimateHistorySeconds);

    private final Random noise = new Random();

    private final double[] heartbeats = new double[ATVisionConstants.LL_IDS.length];
    private final boolean[] cameraConnected = new boolean[ATVisionConstants.LL_IDS.length];

    /** Extra lag added on top of {@link ATVisionConstants#kOffsetTransportLatencySeconds}, for deliberately provoking turret-mismatch rejection in a test. */
    private double extraOffsetLatencySeconds = 0.0;
    /** When set, overrides the computed ambiguity for every tag, for deliberately provoking ambiguity rejection in a test. */
    private Double forcedAmbiguity = null;

    public SimATVision(
            Supplier<SwerveDriveState> swerveState,
            Supplier<Rotation3d> gyroRotation,
            BiConsumer<AprilTagEstimate, Matrix<N3, N1>> addVisionMeasurement,
            Supplier<Double> turretAngleSupplier) {
        super(swerveState, gyroRotation, addVisionMeasurement, turretAngleSupplier);
        this.swerveState = swerveState;
        this.gyroRotation = gyroRotation;
        this.turretAngleSupplier = turretAngleSupplier;
        Arrays.fill(cameraConnected, true);
    }

    /** Test hook: freeze this camera's heartbeat to simulate it dropping off the network. */
    public void setCameraConnected(int cameraIndex, boolean connected) {
        if (cameraIndex >= 0 && cameraIndex < cameraConnected.length) cameraConnected[cameraIndex] = connected;
    }

    /** Test hook: see {@link #extraOffsetLatencySeconds}. */
    public void setExtraOffsetLatencySeconds(double seconds) {
        extraOffsetLatencySeconds = seconds;
    }

    /** Test hook: see {@link #forcedAmbiguity}. Pass {@code null} to go back to the computed model. */
    public void setForcedAmbiguity(Double ambiguity) {
        forcedAmbiguity = ambiguity;
    }

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();

        // Record before publishing this loop's fake frames, mirroring what RealATVision's own
        // (private) pushed-angle history will record this same loop -- see the class Javadoc.
        pushedTurretAngleHistory.addSample(now, Rotation2d.fromDegrees(turretAngleSupplier.get()));

        publishSimulatedCameraFrames(now);

        super.periodic();
    }

    private void publishSimulatedCameraFrames(double now) {
        SwerveDriveState state = swerveState.get();
        Rotation3d gyroRot = gyroRotation.get();
        Rotation3d fieldRotation = new Rotation3d(gyroRot.getX(), gyroRot.getY(), state.Pose.getRotation().getRadians());
        Pose3d fieldToRobot = new Pose3d(new Translation3d(state.Pose.getX(), state.Pose.getY(), 0.0), fieldRotation);

        for (int i = 0; i < ATVisionConstants.LL_IDS.length; i++) {
            String id = ATVisionConstants.LL_IDS[i];

            // A frozen heartbeat is exactly what AprilTagModule watches for to declare a camera
            // disconnected -- so skip everything else for this camera too, same as a real dropout.
            if (!cameraConnected[i]) continue;

            heartbeats[i]++;
            LimelightHelpers.setLimelightNTDouble(id, "hb", heartbeats[i]);

            Pose3d robotToCamera = simulatedCameraOffset(i, now);
            Pose3d fieldToCamera = fieldToRobot.transformBy(
                new Transform3d(robotToCamera.getTranslation(), robotToCamera.getRotation()));

            List<SimObservedTag> visible = findVisibleTags(fieldToCamera);
            LimelightHelpers.setLimelightNTDouble(id, "tv", visible.isEmpty() ? 0.0 : 1.0);

            double mt1XySigma = mt1XYNoiseSigma(visible);
            writeBotposeEntry(id, "botpose_wpiblue",
                state.Pose.getX() + gaussian(mt1XySigma),
                state.Pose.getY() + gaussian(mt1XySigma),
                Math.toDegrees(state.Pose.getRotation().getRadians()) + Math.toDegrees(gaussian(mt1YawNoiseSigma(visible))),
                visible);

            // MegaTag2 never solves its own heading -- it reflects back whatever we seeded it
            // with -- so its simulated yaw carries no independent noise term either.
            writeBotposeEntry(id, "botpose_orb_wpiblue",
                state.Pose.getX() + gaussian(SimVisionConstants.kMT2XYNoiseMeters),
                state.Pose.getY() + gaussian(SimVisionConstants.kMT2XYNoiseMeters),
                Math.toDegrees(fieldRotation.getZ()),
                visible);
        }
    }

    /**
     * The robot-to-camera geometry a real Limelight would actually be using for this frame.
     * <p>
     * For the turret camera this is deliberately NOT the current turret angle: a real camera goes
     * on using whatever camerapose_robotspace it last received until the new write has had time to
     * take effect, which is exactly what {@link ATVisionConstants#kOffsetTransportLatencySeconds}
     * models. Projecting tags with the current angle instead would make {@link RealATVision}'s
     * turret-mismatch check trivially pass every frame, defeating the point of simulating it.
     */
    private Pose3d simulatedCameraOffset(int cameraIndex, double now) {
        if (cameraIndex != ATVisionConstants.kTurretCameraIndex) {
            return ATVisionConstants.LL_OFFSETS[cameraIndex];
        }
        double lookupTime = now - ATVisionConstants.kOffsetTransportLatencySeconds - extraOffsetLatencySeconds;
        Rotation2d usedAngle = pushedTurretAngleHistory.getSample(lookupTime)
            .orElseGet(() -> Rotation2d.fromDegrees(turretAngleSupplier.get()));
        return solveRobotToCamera(usedAngle.getDegrees());
    }

    /** A tag that survived the FOV/range/facing gates, before ambiguity is assigned (which needs the final tag count). */
    private record SimCandidateTag(int id, double rangeMeters, double areaPercent, double viewAngleFrac, double distFrac) {}

    private List<SimObservedTag> findVisibleTags(Pose3d fieldToCamera) {
        List<SimCandidateTag> candidates = new ArrayList<>();
        double halfHFov = Math.toRadians(SimVisionConstants.kHorizontalFOVDeg / 2.0);
        double halfVFov = Math.toRadians(SimVisionConstants.kVerticalFOVDeg / 2.0);

        for (AprilTag tag : fieldLayout.getTags()) {
            Pose3d camToTag = tag.pose.relativeTo(fieldToCamera);
            double x = camToTag.getX();
            if (x <= 0.05) continue; // behind (or right on top of) the camera

            double range = camToTag.getTranslation().getNorm();
            if (range > SimVisionConstants.kMaxTagRangeMeters) continue;

            double yawToTag = Math.atan2(-camToTag.getY(), x);
            double pitchToTag = Math.atan2(camToTag.getZ(), x);
            if (Math.abs(yawToTag) > halfHFov || Math.abs(pitchToTag) > halfVFov) continue;

            // A real camera can't see through the panel the tag is printed on, and the more
            // edge-on the tag appears (as opposed to square-on), the more ambiguous a real
            // pose solve actually is -- this is the foreshortening angle that drives it, not how
            // close to the edge of the camera's own FOV the tag happens to sit.
            Translation3d tagToCamera = fieldToCamera.getTranslation().minus(tag.pose.getTranslation());
            Translation3d tagNormal = new Translation3d(1, 0, 0).rotateBy(tag.pose.getRotation());
            double facingDot = tagToCamera.getX() * tagNormal.getX()
                + tagToCamera.getY() * tagNormal.getY()
                + tagToCamera.getZ() * tagNormal.getZ();
            if (facingDot <= 0) continue;

            double viewAngle = Math.acos(MathUtil.clamp(facingDot / range, -1, 1));
            double viewAngleFrac = viewAngle / (Math.PI / 2); // 0 = square-on, 1 = edge-on
            double distFrac = MathUtil.clamp(range / SimVisionConstants.kMaxTagRangeMeters, 0, 1);
            double areaPercent = MathUtil.clamp(
                SimVisionConstants.kTagAreaAtOneMeterPct / (range * range), 0, 100);

            candidates.add(new SimCandidateTag(tag.ID, range, areaPercent, viewAngleFrac, distFrac));
        }

        // Ambiguity drops sharply with more tags in the solve -- a real multi-tag MegaTag solve is
        // far better constrained than any one of its tags would be alone -- so it has to be scaled
        // after the full candidate count for this frame is known.
        int tagCount = candidates.size();
        List<SimObservedTag> visible = new ArrayList<>(tagCount);
        for (SimCandidateTag c : candidates) {
            double penalty = (SimVisionConstants.kAmbiguityAngleCoeff * c.viewAngleFrac()
                + SimVisionConstants.kAmbiguityDistCoeff * c.distFrac()) / tagCount;
            double ambiguity = forcedAmbiguity != null ? forcedAmbiguity : MathUtil.clamp(
                SimVisionConstants.kBaseAmbiguity + penalty + gaussian(SimVisionConstants.kAmbiguityNoiseStdDev),
                0, 1);
            visible.add(new SimObservedTag(c.id(), c.rangeMeters(), c.areaPercent(), ambiguity));
        }
        return visible;
    }

    private double mt1XYNoiseSigma(List<SimObservedTag> visible) {
        if (visible.isEmpty()) return 0;
        double avgDist = visible.stream().mapToDouble(SimObservedTag::rangeMeters).average().orElse(1);
        return SimVisionConstants.kMT1BaseXYNoiseMeters * (1 + avgDist) / Math.sqrt(visible.size());
    }

    private double mt1YawNoiseSigma(List<SimObservedTag> visible) {
        if (visible.isEmpty()) return 0;
        double avgDist = visible.stream().mapToDouble(SimObservedTag::rangeMeters).average().orElse(1);
        return SimVisionConstants.kMT1BaseYawNoiseRad * (1 + avgDist) / Math.sqrt(visible.size());
    }

    private double gaussian(double sigma) {
        return noise.nextGaussian() * sigma;
    }

    /**
     * Writes one botpose-shaped array in the exact layout {@link AprilTagModule#readPose} expects:
     * {@code [x, y, z, roll, pitch, yaw, latencyMs, tagCount, tagSpan, avgDist, avgArea, (id, txnc,
     * tync, ta, distToCamera, distToRobot, ambiguity) x tagCount]}. With no visible tags this
     * writes the all-zero 11-length array a real Limelight publishes when {@code tv} is false.
     */
    private void writeBotposeEntry(String limelightId, String key, double x, double y, double yawDeg, List<SimObservedTag> visible) {
        int tagCount = visible.size();
        if (tagCount == 0) {
            LimelightHelpers.setLimelightNTDoubleArray(limelightId, key, new double[11]);
            return;
        }

        double[] arr = new double[11 + 7 * tagCount];
        arr[0] = x;
        arr[1] = y;
        arr[5] = yawDeg;
        arr[6] = SimVisionConstants.kSimLatencyMs;
        arr[7] = tagCount;
        arr[9] = visible.stream().mapToDouble(SimObservedTag::rangeMeters).average().orElse(0);
        arr[10] = visible.stream().mapToDouble(SimObservedTag::areaPercent).average().orElse(0);
        for (int t = 0; t < tagCount; t++) {
            SimObservedTag tag = visible.get(t);
            int base = 11 + t * 7;
            arr[base] = tag.id();
            arr[base + 3] = tag.areaPercent();
            arr[base + 4] = tag.rangeMeters();
            arr[base + 5] = tag.rangeMeters();
            arr[base + 6] = tag.ambiguity();
        }
        LimelightHelpers.setLimelightNTDoubleArray(limelightId, key, arr);
    }
}
