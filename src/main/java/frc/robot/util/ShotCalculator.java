package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.ShotController.ShooterTargetData;

public class ShotCalculator {

    private final Supplier<Pose2d> robotPose;
    private final Supplier<ChassisSpeeds> robotSpeeds;
    private final Supplier<Pose2d> hubPose;

    //from the shot calculator 
    private final double[] angle_coefficients = {0.06819563819722342, -3.0312156212267225, 86.57708128244042};
    private final double[] velocity_coefficients = {-0.03970191850324918, 0.6023478186630093, 8.241406861728594};

    private final InterpolatingDoubleTreeMap velocityToRpmMap = new InterpolatingDoubleTreeMap();
    
    //not sure if the hubPose needs to be a supplier it could just be a pose2d i think
    public ShotCalculator(Supplier<Pose2d> robotPose, 
                          Supplier<ChassisSpeeds> robotSpeeds, 
                          Supplier<Pose2d> hubPose) {

        this.robotPose = robotPose;
        this.robotSpeeds = robotSpeeds;
        this.hubPose = hubPose;
    }

    public ShooterTargetData calculateShot() {
        return null;
    }

    public double calculateFlywheelRPM() {
        return 0;
    }

    public double calculateHoodAngle() {
        return 0;
    }

    public double calculateTurretAngle() {
        return 0;
    }
}
