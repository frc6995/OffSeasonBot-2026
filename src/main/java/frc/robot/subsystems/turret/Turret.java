package frc.robot.subsystems.turret;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
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
    // The angle actually sent to the IO this loop, for telemetry (DISABLED leaves this at its last value).
    private double commandedAngle = 0;

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
            case AIM_CENTRAL -> commandedAngle = selectCentralAngle(shotData.get().turretAngleDeg());
            case AIM_CLOSEST -> commandedAngle = selectClosestAngle(shotData.get().turretAngleDeg());
            case MANUAL -> commandedAngle = selectClosestAngle(requestedAngle);
        }

        io.updateInputs(inputs);
    }

    @Override
    public void simulationPeriodic() {
        turretLigament.setAngle(inputs.angle);
        RobotVisualizer.updateTurret(Units.degreesToRadians(inputs.angle));
    }

    public void setState(TurretState state) {
        turretState = state;
    }

    // just for testing in sim
    public void setAngle(double angle) {
        requestedAngle = angle;

        this.turretState = TurretState.MANUAL;
    }

    private double selectClosestAngle(double angle) {
        double currentAngle = this.getAngle();

        angle = MathUtil.inputModulus(angle, -180, 180);

        double smallestAngle = angle;
        double smallestDifference = Math.abs(angle - currentAngle);

        if (angle >= 0) {
            double alt = angle - 360;
            double diff = Math.abs(alt - currentAngle);
            if (diff < smallestDifference) {
                smallestDifference = diff;
                smallestAngle = alt;
            }
        }

        if (angle <= 0) {
            double alt = angle + 360;
            double diff = Math.abs(alt - currentAngle);
            if (diff < smallestDifference) {
                smallestDifference = diff;
                smallestAngle = alt;
            }
        }

        io.setAngle(smallestAngle);
        return smallestAngle;
    }

    private double selectCentralAngle(double angle) {
        angle = MathUtil.inputModulus(angle, -180, 180);

        io.setAngle(angle);
        return angle;
    }

    @Logged(name = "State", importance = Importance.CRITICAL)
    public TurretState getState() {
        return turretState;
    }

    @Logged(name = "Connected", importance = Importance.CRITICAL)
    public boolean isConnected() {
        return inputs.turretMotorConnected;
    }

    @Logged(name = "Angle", importance = Importance.INFO)
    public double getAngle() {
        return inputs.angle;
    }

    @Logged(name = "Setpoint", importance = Importance.INFO)
    public double getRequestedAngle() {
        return commandedAngle;
    }

    @Logged(name = "Stator Current", importance = Importance.DEBUG)
    public double getStatorCurrent() {
        return inputs.statorCurrent;
    }

    @Logged(name = "Supply Current", importance = Importance.DEBUG)
    public double getSupplyCurrent() {
        return inputs.supplyCurrent;
    }

    @Logged(name = "Voltage", importance = Importance.DEBUG)
    public double getVoltage() {
        return inputs.appliedVolts;
    }
}
