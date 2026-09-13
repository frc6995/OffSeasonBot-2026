package frc.robot.util;

import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.ShotController.ShooterTargetData;

public class ShotCalculator {

    private final Supplier<Pose2d> robotPose;
    private final Supplier<Rotation2d> robotAngle;
    private final Supplier<ChassisSpeeds> robotSpeeds;
    private final Supplier<Pose2d> hubPose;

    //from the shot calculator https://github.com/Maro1810/FRC-Shot-Calculator
    private final double[] angle_coefficients = {0.06819563819722342, -3.0312156212267225, 86.57708128244042};
    private final double[] vel_coefficients = {-0.03970191850324918, 0.6023478186630093, 8.241406861728594};

    private final Function<Double, Double> angle_function = (x) -> {
        return angle_coefficients[0]*(x*x)+angle_coefficients[1]*(x)+angle_coefficients[2];
    };

    private final Function<Double, Double> velocity_function = (x) -> {
        return vel_coefficients[0]*(x*x)+vel_coefficients[1]*(x)+vel_coefficients[2];
    };

    private final InterpolatingDoubleTreeMap velocityToRpmMap = new InterpolatingDoubleTreeMap();
    
    //not sure if the hubPose needs to be a supplier it could just be a pose2d i think
    public ShotCalculator(Supplier<Pose2d> robotPose, 
                          Supplier<Rotation2d> robotAngle,
                          Supplier<ChassisSpeeds> robotSpeeds, 
                          Supplier<Pose2d> hubPose) {

        this.robotPose = robotPose;
        this.robotAngle = robotAngle;
        this.robotSpeeds = robotSpeeds;
        this.hubPose = hubPose;
    }

    public ShooterTargetData calculateShot() {
        return new ShooterTargetData(
            calculateFlywheelRPM(), 
            calculateHoodAngle(), 
            calculateTurretAngle());
    }

    private double calculateFlywheelRPM() {
        return 0;
    }

    private double calculateHoodAngle() {
        return 0;
    }

    private double calculateTurretAngle() {
        double xDisplacement = (hubPose.get().getX() - robotPose.get().getX());
        double yDisplacement = (hubPose.get().getY() - robotPose.get().getY());

        double initialAngle = Math.atan2(yDisplacement, xDisplacement);

        double distance = hubPose.get().getTranslation().getDistance(robotPose.get().getTranslation());

        return 0;
    }
}
