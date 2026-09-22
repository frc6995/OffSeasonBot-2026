package frc.robot.util;

import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.util.ShotController.ShooterTargetData;
import frc.robot.util.ShotController.ShotConstants;

public class ShotCalculator {

    private final Supplier<Pose2d> robotPose;
    private final Supplier<ChassisSpeeds> robotSpeeds;
    private final Supplier<Pose2d> hubPose;

    private ShooterTargetData cachedData = new ShooterTargetData(0, 0, 0);

    //from the shot calculator https://github.com/Maro1810/FRC-Shot-Calculator
    private final double[] angle_coefficients = {-1.4712590406264083, 6.0689490309911935, 64.11638838409094};
    private final double[] vel_coefficients = {-0.1526463705946785, 1.6055924674659063, 4.314999775236604};

    private double LATENCY_SECONDS = 0.02;

    private final Function<Double, Double> angle_function = (x) -> {
        return angle_coefficients[0]*(x*x)+angle_coefficients[1]*(x)+angle_coefficients[2];
    };

    private final Function<Double, Double> velocity_function = (x) -> {
        return vel_coefficients[0]*(x*x)+vel_coefficients[1]*(x)+vel_coefficients[2];
    };

    // private final InterpolatingDoubleTreeMap velocityToRpmMap = new InterpolatingDoubleTreeMap();
    private final Function<Double,Double> velocityToRpmFunc = (v) -> v*310.1924977;

    private final DoublePublisher m_distancePublisher;
    
    //not sure if the hubPose needs to be a supplier it could just be a pose2d i think
    //THIS REQUIRES FIELD RELATIVE ROBOT SPEEDS
    public ShotCalculator(Supplier<Pose2d> robotPose, 
                          Supplier<ChassisSpeeds> robotSpeeds, 
                          Supplier<Pose2d> hubPose) {

        this.robotPose = robotPose;
        this.robotSpeeds = robotSpeeds;
        this.hubPose = hubPose;

        var table = NetworkTableInstance.getDefault().getTable("ShotCalculator");
        m_distancePublisher = table.getDoubleTopic("Target Distance").publish();
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

        double xDisplacement = (predictedRobotPose.getX() - goalPose.getX());
        double yDisplacement = (predictedRobotPose.getY() - goalPose.getY());

        double initialTurretAngle = Math.PI - Math.atan2(yDisplacement, xDisplacement);

        double distance = Math.sqrt(Math.pow(xDisplacement, 2) + Math.pow(yDisplacement, 2));

        if(ShotConstants.kShouldLog) {
            m_distancePublisher.accept(distance);
        }

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

        double launchRPM = velocityToRpmFunc.apply(correctedLaunchSpeed);

        cachedData = new ShooterTargetData(launchRPM, correctedHoodAngle, correctedTurretAngle);
        
        return cachedData;
    }

    public ShooterTargetData getCachedData() {
        return cachedData;
    }
}
