package frc.robot.util;

import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.ShotController.ShooterTargetData;

public class ShotCalculator {

    private final Supplier<Pose2d> robotPose;
    private final Supplier<ChassisSpeeds> robotSpeeds;
    private final Supplier<Pose2d> hubPose;

    private ShooterTargetData cachedData = new ShooterTargetData(0, 0, 0);

    //from the shot calculator https://github.com/Maro1810/FRC-Shot-Calculator
    private final double[] angle_coefficients = {0.06819563819722342, -3.0312156212267225, 86.57708128244042};
    private final double[] vel_coefficients = {-0.03970191850324918, 0.6023478186630093, 8.241406861728594};

    private double LATENCY_SECONDS = 0.02;

    private final Function<Double, Double> angle_function = (x) -> {
        return angle_coefficients[0]*(x*x)+angle_coefficients[1]*(x)+angle_coefficients[2];
    };

    private final Function<Double, Double> velocity_function = (x) -> {
        return vel_coefficients[0]*(x*x)+vel_coefficients[1]*(x)+vel_coefficients[2];
    };

    private final InterpolatingDoubleTreeMap velocityToRpmMap = new InterpolatingDoubleTreeMap();
    
    //not sure if the hubPose needs to be a supplier it could just be a pose2d i think
    //THIS REQUIRES FIELD RELATIVE ROBOT SPEEDS
    public ShotCalculator(Supplier<Pose2d> robotPose, 
                          Supplier<ChassisSpeeds> robotSpeeds, 
                          Supplier<Pose2d> hubPose) {

        this.robotPose = robotPose;
        this.robotSpeeds = robotSpeeds;
        this.hubPose = hubPose;
    }

    public ShooterTargetData calculateShot() {
        ChassisSpeeds currentSpeeds = robotSpeeds.get();
        Pose2d goalPose = hubPose.get();
        Pose2d currentPose = robotPose.get();

        Pose2d predictedRobotPose = new Pose2d(
            currentPose.getX()+currentSpeeds.vxMetersPerSecond*LATENCY_SECONDS,
            currentPose.getY()+currentSpeeds.vyMetersPerSecond*LATENCY_SECONDS,
            currentPose.getRotation().plus(new Rotation2d(currentSpeeds.omegaRadiansPerSecond*LATENCY_SECONDS))
        );

        double xDisplacement = (goalPose.getX() - predictedRobotPose.getX());
        double yDisplacement = (goalPose.getY() - predictedRobotPose.getY());

        double initialTurretAngle = Math.atan2(yDisplacement, xDisplacement);

        double distance = goalPose.getTranslation().getDistance(predictedRobotPose.getTranslation());

        double speed = velocity_function.apply(distance);
        double initialHoodAngle = angle_function.apply(distance);

        double horiz_speed = speed*Math.cos(initialHoodAngle);

        Translation2d v_horizontal = new Translation2d(horiz_speed*Math.cos(initialTurretAngle), horiz_speed*Math.sin(initialTurretAngle));
        Translation2d v_robot = new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

        Translation2d v_horizontal_new = v_horizontal.minus(v_robot);

        double correctedTurretAngle = v_horizontal_new.getAngle().getDegrees() - predictedRobotPose.getRotation().getDegrees();

        double correctedLaunchSpeed = Math.sqrt(
            Math.pow(v_horizontal_new.getNorm(), 2)+
            Math.pow(speed*Math.sin(initialHoodAngle), 2));

        double correctedHoodAngle = Math.toDegrees(
            Math.atan2(speed*Math.sin(initialHoodAngle), v_horizontal_new.getNorm())
        );

        double launchRPM = velocityToRpmMap.get(correctedLaunchSpeed);

        cachedData = new ShooterTargetData(launchRPM, correctedHoodAngle, correctedTurretAngle);
        
        return cachedData;
    }

    public ShooterTargetData getCachedData() {
        return cachedData;
    }
}
