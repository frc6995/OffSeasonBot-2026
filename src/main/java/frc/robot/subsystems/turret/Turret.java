package frc.robot.subsystems.turret;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;

public class Turret extends SubsystemBase {

    private TurretState state = TurretState.DISABLED;
    private double requestedAngle;
    private TurretIO io;
    private TurretIOInputs inputs = new TurretIOInputs();

    static class TurretConstants {
        public static int kCANID = 45; 

        //Tune PID/FF constants
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;

        public static final double kSimP = 0;
        public static final double kSimI = 0;
        public static final double kSimD = 0;
        public static final double kSimS = 0;
        public static final double kSimV = 0;
        public static final double kSimA = 0;

        public static final double kStatorCurrentLimitAmps = 80;
        public static final double kSupplyCurrentLimitAmps = 40;

        public static final double kMinAngle = -360;
        public static final double kMaxAngle = 360;

        public static final double kReduction = 32.5;

        public static final double kMOI = 0.0873236726;

        //6.5 in
        public static final double kLength = 0.1651;
    }

    public enum TurretState {
        DISABLED,
        AIM_CLOSEST,
        AIM_CENTRAL
    }

    public Turret(TurretIO io) {
        this.io = io;
    }

    public void aimClosest() {
        state = TurretState.AIM_CLOSEST;
    }

    public void aimCentral() {
        state = TurretState.AIM_CENTRAL;
    }

    public void disable() {
        state = TurretState.DISABLED;
    }

    @Override
    public void periodic() {
        switch (state) {
            case DISABLED -> io.disable();

            // need to fix this because currently this will only command 0 degrees
            case AIM_CENTRAL -> selectCentralAngle(requestedAngle);
            case AIM_CLOSEST -> selectClosestAngle(requestedAngle);
        }

        io.updateInputs(inputs);
    }

    public TurretState getState() {
        return state;
    }

    public double getAngle() {
        return inputs.angle;
    }

    public double getRequestedAngle() {
        return requestedAngle;
    }

    public void selectClosestAngle(double angle) {
        double currentAngle = this.getAngle();

        angle = MathUtil.inputModulus(angle, -180, 180);

        ArrayList<Double> possibleAngles = new ArrayList<>(2);

        possibleAngles.add(angle);

        if (angle >= 0) {
            possibleAngles.add(angle-360);
        }

        if (angle <= 0) {
            possibleAngles.add(angle+360);
        }

        double smallestAngle = angle;
        double smallestDifference = Math.abs(angle-currentAngle);

        for (int i = 1; i < possibleAngles.size(); i++) {
            double diff = Math.abs(possibleAngles.get(i)-currentAngle);

            if (diff < smallestDifference) {
                smallestDifference = diff;
                smallestAngle = possibleAngles.get(i);
            }
        }

        io.setAngle(smallestAngle);
    }

    public void selectCentralAngle(double angle) {
        angle = MathUtil.inputModulus(angle, -180, 180);

        io.setAngle(angle);
    }
}
