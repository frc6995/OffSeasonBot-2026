package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;

public class Turret extends SubsystemBase{

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

        public static final double kStatorCurrentLimitAmps = 0;
        public static final double kSupplyCurrentLimitAmps = 0;

        public static final double kMinAngle = -360;
        public static final double kMaxAngle = 360;

        public static final double kReduction = 32.5;

        public static final double kMOI = 0.0873236726;

        //6.5 in
        public static final double kLength = 0.1651;
    }

    public enum TurretState {
        DISABLED,
        POSITION
    }

    public Turret(TurretIO io) {
        this.io = io;
    }

    public void setAngle(double angle) {
        requestedAngle = angle;

        state = TurretState.POSITION;
    }

    public void disable() {
        state = TurretState.DISABLED;
    }

    @Override
    public void periodic() {
        switch (state) {
            case DISABLED -> io.disable();
            case POSITION -> io.setAngle(requestedAngle);
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
}
