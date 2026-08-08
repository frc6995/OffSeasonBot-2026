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
        public static final double kTurretVelocityComp = 5.0;
        public static final double kFlywheelVelocityComp = 0.0;
        public static final double kHoodVelocityComp = 0.0;
    }

    public static record ShooterTargetData(double flywheelRpm, double hoodAngleDeg, double turretAngleDeg) {}

    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();

    private final Supplier<Pose2d> robotPose;
    private final Supplier<ChassisSpeeds> robotSpeeds;
    private final Supplier<Pose2d> goalPose;

    private ShooterTargetData cachedData = new ShooterTargetData(0, 0, 0);

    public ShotController(
        Supplier<Pose2d> robotPose,
        Supplier<ChassisSpeeds> robotSpeeds,
        Supplier<Pose2d> goalPose
    ) {
        this.robotPose = robotPose;
        this.robotSpeeds = robotSpeeds;
        this.goalPose = goalPose;
        populateLUTs();
    }

    private void populateLUTs() {

        for(var value : FlywheelConstants.kShooterData) {
            rpmMap.put(value[0], value[1]);
        }

        for(var value : HoodConstants.kAngleData) {
            hoodMap.put(value[0], value[1]);
        }
    }

    public ShooterTargetData calculate() {
        Pose2d currentPose = robotPose.get();
        Pose2d targetPose = goalPose.get();

        Translation2d robotToGoal = targetPose.getTranslation().minus(currentPose.getTranslation());

        double[] polarSpeeds = convertToTargetPolar(robotToGoal, robotSpeeds.get());

        ShooterTargetData targetData = new ShooterTargetData(
            calculateFlywheelRpm(robotToGoal, polarSpeeds[1]),
            calculateHoodAngle(robotToGoal, polarSpeeds[1]), 
            calculateTurretAngle(currentPose.getRotation(), robotToGoal, polarSpeeds[0]));

        cachedData = targetData;
        return cachedData;
    }

    public ShooterTargetData getCachedData() {
        return cachedData;
    }

    private double calculateTurretAngle(Rotation2d robotAngle, Translation2d robotToGoal, double targetTanVelocity) {
        double baseAngle = robotToGoal.getAngle().getDegrees() + robotAngle.getDegrees();
        double velocityComp = targetTanVelocity * ShotConstants.kTurretVelocityComp;
        return baseAngle + velocityComp;
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

    private double[] convertToTargetPolar(Translation2d robotToGoal, ChassisSpeeds speeds) {
        Translation2d rHat = robotToGoal.div(robotToGoal.getNorm());
        Translation2d tHat = new Translation2d(-rHat.getY(), rHat.getX());

        double vRadial = speeds.vxMetersPerSecond * rHat.getX() + speeds.vyMetersPerSecond * rHat.getY();
        double vTangential = speeds.vxMetersPerSecond * tHat.getX() + speeds.vyMetersPerSecond * tHat.getY();

        return new double[] {vRadial, vTangential};
    }
}
