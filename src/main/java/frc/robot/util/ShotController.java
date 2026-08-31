package frc.robot.util;

import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.flywheel.Flywheel.FlywheelConstants;
import frc.robot.subsystems.hood.Hood.HoodConstants;
public class ShotController {
    public static final class ShotConstants {
        /**
         * Turret shoot-on-the-move gain, in degrees of aim-off per m/s of tangential speed.
         * POSITIVE; {@link ShotController#calculateTurretAngle} subtracts it.
         */
        public static final double kTurretVelocityComp = 20.0;

        /**
         * Flywheel shoot-on-the-move gain, in RPM per m/s of radial (closing) speed. Currently
         * inert. If enabled, note that closing on the goal ADDS speed to the ball, so this must be
         * NEGATIVE to reduce RPM -- {@link ShotController#calculateFlywheelRpm} adds it directly.
         */
        public static final double kFlywheelVelocityComp = 0.0;

        /**
         * Hood shoot-on-the-move gain, in degrees per m/s of radial (closing) speed. Currently
         * inert. Same sign caution as {@link #kFlywheelVelocityComp}: closing on the goal makes the
         * ball travel further, so this must be NEGATIVE to flatten the shot.
         */
        public static final double kHoodVelocityComp = 0.0;
    }

    public static record ShooterTargetData(double flywheelRpm, double hoodAngleDeg, double turretAngleDeg) {}

    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap passingRpmMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap passingHoodMap = new InterpolatingDoubleTreeMap();

    private final Supplier<Pose2d> robotPose;
    private final Supplier<ChassisSpeeds> robotSpeeds;
    private final Supplier<Pose2d> goalPose;
    private final Supplier<Rotation2d> passingAngle;
    private final Supplier<Translation2d> passingWallStart;
    private final Supplier<Translation2d> passingWallEnd;

    private ShooterTargetData cachedData = new ShooterTargetData(0, 0, 0);

    public ShotController(
        Supplier<Pose2d> robotPose,
        Supplier<ChassisSpeeds> robotSpeeds,
        Supplier<Pose2d> goalPose,
        Supplier<Rotation2d> passingAngle,
        Supplier<Translation2d> passingWallStart,
        Supplier<Translation2d> passingWallEnd
    ) {
        this.robotPose = robotPose;
        this.robotSpeeds = robotSpeeds;
        this.goalPose = goalPose;
        this.passingAngle = passingAngle;
        this.passingWallStart = passingWallStart;
        this.passingWallEnd = passingWallEnd;
        populateLUTs();
    }

    private void populateLUTs() {

        for(var value : FlywheelConstants.kShooterData) {
            rpmMap.put(value[0], value[1]);
        }

        for(var value : HoodConstants.kAngleData) {
            hoodMap.put(value[0], value[1]);
        }

        for(var value : FlywheelConstants.kPassingShooterData) {
            passingRpmMap.put(value[0], value[1]);
        }

        for(var value : HoodConstants.kPassingAngleData) {
            passingHoodMap.put(value[0], value[1]);
        }
    }

    public ShooterTargetData calculate() {
        return calculate(false);
    }

    /**
     * @param isPassing when true, the turret points at the fixed {@link POI#PASSING_ANGLE}
     *                  field-relative heading instead of tracking {@code goalPose}.
     */
    public ShooterTargetData calculate(boolean isPassing) {
        Pose2d currentPose = robotPose.get();

        ShooterTargetData targetData = isPassing
            ? calculatePassingData(currentPose)
            : calculateScoringData(currentPose);

        cachedData = targetData;
        return cachedData;
    }

    private ShooterTargetData calculateScoringData(Pose2d currentPose) {
        Pose2d targetPose = goalPose.get();

        Translation2d robotToGoal = targetPose.getTranslation().minus(currentPose.getTranslation());

        // robotSpeeds is robot-centric (Phoenix documents SwerveDriveState.Speeds as
        // "the current robot-centric velocity"), but rHat/tHat below are built from robotToGoal
        // and are therefore field-relative. Convert before projecting, or the decomposition is
        // wrong for every heading except zero, and wrong by 90 degrees at a quarter turn.
        ChassisSpeeds fieldSpeeds =
            ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds.get(), currentPose.getRotation());

        TargetPolarSpeeds polarSpeeds = convertToTargetPolar(robotToGoal, fieldSpeeds);

        return new ShooterTargetData(
            calculateFlywheelRpm(robotToGoal, polarSpeeds.radial()),
            calculateHoodAngle(robotToGoal, polarSpeeds.radial()),
            calculateTurretAngle(currentPose.getRotation(), robotToGoal, polarSpeeds.tangential()));
    }

    private ShooterTargetData calculatePassingData(Pose2d currentPose) {
        double wallDistance = distanceFromLine(
            currentPose.getTranslation(), passingWallStart.get(), passingWallEnd.get());

        return new ShooterTargetData(
            passingRpmMap.get(wallDistance),
            passingHoodMap.get(wallDistance),
            calculatePassingTurretAngle(currentPose.getRotation()));
    }

    public ShooterTargetData getCachedData() {
        return cachedData;
    }

    /**
     * This converts the field-relative {@link POI#PASSING_ANGLE} into a robot-relative
     * angle, the same way {@link #calculateTurretAngle} does.
     */
    private double calculatePassingTurretAngle(Rotation2d robotAngle) {
        return passingAngle.get().getDegrees() - robotAngle.getDegrees();
    }

    /**
     * Turret angle in the robot frame, CCW positive, with a correction for the robot's own motion.
     *
     * <p>The ball leaves with the shooter's exit velocity PLUS the robot's velocity, so a robot
     * moving counter-clockwise about the goal (positive tangential) sends the ball drifting
     * counter-clockwise of it. Cancelling that means aiming clockwise -- <i>subtracting</i> from the
     * base bearing, not adding to it.
     *
     * <p>This term previously added, which steered further into the drift and roughly doubled the
     * error instead of removing it. It was masked until now: the tangential component was never
     * actually reaching this method (the polar components were swapped) and the speeds feeding it
     * were in the wrong frame, so the number here was unrelated to lateral motion either way.
     *
     * <p>{@link ShotConstants#kTurretVelocityComp} is therefore a POSITIVE gain in degrees per m/s.
     * Its magnitude has never been validated on hardware -- see the constant.
     */
    private double calculateTurretAngle(Rotation2d robotAngle, Translation2d robotToGoal, double targetTanVelocity) {
        double baseAngle = robotToGoal.getAngle().getDegrees() - robotAngle.getDegrees();
        double velocityComp = targetTanVelocity * ShotConstants.kTurretVelocityComp;
        return baseAngle - velocityComp;
    }

    private double calculateHoodAngle(Translation2d robotToGoal, double targetRadialVelocity) {
        double baseAngle = hoodMap.get(robotToGoal.getNorm());
        double velocityComp = targetRadialVelocity * ShotConstants.kHoodVelocityComp;
        return baseAngle + velocityComp;
    }

    private double calculateFlywheelRpm(Translation2d robotToGoal, double targetRadialVelocity) {
        double baseSpeed = rpmMap.get(robotToGoal.getNorm());
        double velocityComp = targetRadialVelocity * ShotConstants.kFlywheelVelocityComp;
        return baseSpeed + velocityComp;
    }

    /**
     * Field-relative robot velocity decomposed about the goal. {@code radial} is closing speed
     * (positive means approaching); {@code tangential} is lateral speed about the goal, positive
     * counter-clockwise.
     *
     * <p>A record rather than a {@code double[]}: these two were previously returned as an array
     * and read back by index at three call sites, all of which had the indices swapped.
     */
    private record TargetPolarSpeeds(double radial, double tangential) {}

    /**
     * @param speeds FIELD-relative speeds. Passing robot-relative speeds here silently produces a
     *               wrong decomposition rather than an error -- see {@link #calculateScoringData}.
     */
    private TargetPolarSpeeds convertToTargetPolar(Translation2d robotToGoal, ChassisSpeeds speeds) {
        double distance = robotToGoal.getNorm();
        if (distance < 1e-6) {
            // Degenerate: sitting exactly on the goal leaves the radial direction undefined, and
            // dividing through would poison every downstream setpoint with NaN.
            return new TargetPolarSpeeds(0.0, 0.0);
        }

        Translation2d rHat = robotToGoal.div(distance);
        Translation2d tHat = new Translation2d(-rHat.getY(), rHat.getX());

        double vRadial = speeds.vxMetersPerSecond * rHat.getX() + speeds.vyMetersPerSecond * rHat.getY();
        double vTangential = speeds.vxMetersPerSecond * tHat.getX() + speeds.vyMetersPerSecond * tHat.getY();

        return new TargetPolarSpeeds(vRadial, vTangential);
    }

    /** Perpendicular distance from {@code point} to the infinite line through {@code lineStart}/{@code lineEnd}. */
    private double distanceFromLine(Translation2d point, Translation2d lineStart, Translation2d lineEnd) {
        Translation2d lineVec = lineEnd.minus(lineStart);
        Translation2d pointVec = point.minus(lineStart);

        double cross = lineVec.getX() * pointVec.getY() - lineVec.getY() * pointVec.getX();

        return Math.abs(cross) / lineVec.getNorm();
    }
}
