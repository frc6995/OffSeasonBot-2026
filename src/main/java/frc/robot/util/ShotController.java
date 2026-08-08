package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.flywheel.Flywheel.FlywheelConstants;
import frc.robot.subsystems.hood.Hood.HoodConstants;
public class ShotController {

    public static record ShooterTargetData(double rpm, double hoodAngle) {}

    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();

    private final Supplier<Pose2d> robotPose;
    private final Supplier<ChassisSpeeds> robotSpeeds;
    private final Supplier<Pose2d> goalPose;

    private ShooterTargetData cachedData = new ShooterTargetData(0, 0);

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
        Pose2d goPose = goalPose.get();

        double distance = currentPose.getTranslation().getDistance(goPose.getTranslation());

        ShooterTargetData targetData = new ShooterTargetData(rpmMap.get(distance), hoodMap.get(distance));

        cachedData = targetData;
        return cachedData;
    }

    public ShooterTargetData getCachedData() {
        return cachedData;
    }
}
