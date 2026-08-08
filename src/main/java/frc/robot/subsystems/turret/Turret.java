package frc.robot.subsystems.turret;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotVisualizer;
import frc.robot.subsystems.hood.Hood.HoodState;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;

public class Turret extends SubsystemBase {

    private TurretState turretState = TurretState.DISABLED;
    private double requestedAngle;
    private TurretIO io;
    private TurretIOInputs inputs = new TurretIOInputs();
    private final MechanismLigament2d turretLigament = new MechanismLigament2d("turret", Units.inchesToMeters(12), 0, 6,
        new Color8Bit(137, 52, 235));

    static class TurretConstants {
        public static int kCANID = 45;

        // Tune PID/FF constants
        public static final double kP = 30;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;

        public static final double kStatorCurrentLimitAmps = 80;
        public static final double kSupplyCurrentLimitAmps = 40;

        public static final double kMinAngle = -360;
        public static final double kMaxAngle = 360;

        public static final double kReduction = 32.5;

        public static final double kMOI = 0.0873236726;

        // 6.5 in
        public static final double kLength = 0.1651;
    }

    public enum TurretState {
        DISABLED,
        AIM_CLOSEST,
        AIM_CENTRAL
    }

    public Turret(TurretIO io) {
        this.io = io;
        RobotVisualizer.addTurret(turretLigament);
    }

    public void requestAimClosest() {
        turretState = TurretState.AIM_CLOSEST;
    }

    public void requestAimCentral() {
        turretState = TurretState.AIM_CENTRAL;
    }

    public void requestDisable() {
        turretState = TurretState.DISABLED;
    }

    @Override
    public void periodic() {
        switch (turretState) {
            case DISABLED -> io.disable();

            // need to fix this because currently this will only command 0 degrees
            case AIM_CENTRAL -> selectCentralAngle(requestedAngle);
            case AIM_CLOSEST -> selectClosestAngle(requestedAngle);
        }

        io.updateInputs(inputs);
        turretLigament.setAngle(inputs.angle);

    }

    public TurretState getState() {
        return turretState;
    }

    public void setState(TurretState state) {
        turretState = state;
    }

    public double getAngle() {
        return inputs.angle;
    }

    // just for testing in sim
    public void setAngle(double angle) {
        requestedAngle = angle;

        this.turretState = TurretState.AIM_CENTRAL;
    }

    public double getRequestedAngle() {
        return requestedAngle;
    }

    private void selectClosestAngle(double angle) {
        double currentAngle = this.getAngle();

        angle = MathUtil.inputModulus(angle, -180, 180);

        ArrayList<Double> possibleAngles = new ArrayList<>(2);

        possibleAngles.add(angle);

        if (angle >= 0) {
            possibleAngles.add(angle - 360);
        }

        if (angle <= 0) {
            possibleAngles.add(angle + 360);
        }

        double smallestAngle = angle;
        double smallestDifference = Math.abs(angle - currentAngle);

        for (int i = 1; i < possibleAngles.size(); i++) {
            double diff = Math.abs(possibleAngles.get(i) - currentAngle);

            if (diff < smallestDifference) {
                smallestDifference = diff;
                smallestAngle = possibleAngles.get(i);
            }
        }

        io.setAngle(smallestAngle);
    }

    private void selectCentralAngle(double angle) {
        angle = MathUtil.inputModulus(angle, -180, 180);

        io.setAngle(angle);
    }
}
