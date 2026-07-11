package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hood.HoodIO.HoodIOInputs;

public class Hood extends SubsystemBase{

    private HoodIO io;
    private HoodIOInputs hoodIOInputs = new HoodIOInputs();

    private double requestedAngle;

    private HoodState state = HoodState.DISABLED;

    static class HoodConstants {
        public static int kCANID = 0; //have to figure this out

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

        public static final double kReduction = 70.2857;

        public static final double MIN_ANGLE = 0;
        public static final double MAX_ANGLE = 42.5;

        //Originally 11.5 in^2 lbs, this is in kg m^2
        public static final double kMOI = 0.00336535601;

        //5.57 inches
        public static final double kHoodLength = 0.141478;

    }

    public enum HoodState {
        DISABLED,
        POSITION
    }

    public Hood(HoodIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(hoodIOInputs);

        switch(state) {
            case DISABLED:
                io.disable();
            case POSITION:

                double clampedAngle = MathUtil.clamp(requestedAngle, 0, Hood.HoodConstants.MAX_ANGLE);

                io.setAngle(clampedAngle);

        }
    }

    public void disable() {
        state = HoodState.DISABLED;
    }

    public double getRequestedAngle() {
        return requestedAngle;
    }

    public HoodState getState() {
        return state;
    }

    public double getAngle() {
        return hoodIOInputs.angle;
    }

    public void setAngle(double angle) {
        state = HoodState.POSITION;

        requestedAngle = angle;
    }
}
