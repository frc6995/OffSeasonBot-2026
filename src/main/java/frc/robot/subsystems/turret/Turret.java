package frc.robot.subsystems.turret;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotVisualizer;
import frc.robot.subsystems.hood.Hood.HoodState;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;
import frc.robot.util.ShotController.ShooterTargetData;

public class Turret extends SubsystemBase {
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
        AIM_CENTRAL,
        MANUAL;
    }

    private TurretState turretState = TurretState.AIM_CLOSEST;
    private double requestedAngle = 0;

    private TurretIO io;
    private Supplier<ShooterTargetData> shotData;

    private TurretIOInputs inputs = new TurretIOInputs();

    private final MechanismLigament2d turretLigament = new MechanismLigament2d("turret", Units.inchesToMeters(12), 0, 6,
        new Color8Bit(137, 52, 235));

    public Turret(TurretIO io, Supplier<ShooterTargetData> shotData) {
        this.io = io;
        this.shotData = shotData;
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
            case AIM_CENTRAL -> selectCentralAngle(shotData.get().turretAngleDeg());
            case AIM_CLOSEST -> selectClosestAngle(shotData.get().turretAngleDeg());
            case MANUAL -> selectClosestAngle(requestedAngle);
        }

        io.updateInputs(inputs);
    }

    @Override
    public void simulationPeriodic() {
        turretLigament.setAngle(inputs.angle);
        RobotVisualizer.updateTurret(Units.degreesToRadians(inputs.angle));
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

        this.turretState = TurretState.MANUAL;
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
