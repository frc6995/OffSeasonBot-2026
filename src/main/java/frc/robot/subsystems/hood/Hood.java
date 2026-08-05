package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotVisualizer;
import frc.robot.subsystems.hood.HoodIO.HoodIOInputs;

public class Hood extends SubsystemBase {

    private HoodIO io;
    private HoodIOInputs hoodIOInputs = new HoodIOInputs();

    private final MechanismLigament2d hoodLigament = new MechanismLigament2d("hood", Units.inchesToMeters(4), 0, 6,
            new Color8Bit(52, 137, 235));

    private double requestedAngle;

    private HoodState hoodState = HoodState.DISABLED;

    static class HoodConstants {
        public static int kCANID = 44; // Should be right with doc

        //Tune PID/FF constants
        public static final double kP = 120; //Double check this
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kV = 0.1;
        public static final double kG = 0;

        public static final double kStatorCurrentLimitAmps = 80;
        public static final double kSupplyCurrentLimitAmps = 40;

        public static final double kReduction = 70.2857;

        public static final double MIN_ANGLE = 0;
        public static final double MAX_ANGLE = 42.5;

        // Originally 11.5 in^2 lbs, this is in kg m^2
        public static final double kMOI = 0.00336535601;

        // 5.57 inches
        public static final double kHoodLength = 0.141478;

    }

    public enum HoodState {
        DISABLED,
        ACTIVE
    }

    public Hood(HoodIO io) {
        this.io = io;
        RobotVisualizer.addHood(hoodLigament);
    }
    

    @Override
    public void periodic() {
        io.updateInputs(hoodIOInputs);

        switch (hoodState) {
            case DISABLED:
                io.disable();
                break;
            case ACTIVE:

                double clampedAngle = applyLimits(requestedAngle);

                io.setAngle(clampedAngle);

        }

        hoodLigament.setAngle(getAngle());
    }

    public void disable() {
        hoodState = HoodState.DISABLED;
    }

    public double getRequestedAngle() {
        return requestedAngle;
    }

    public HoodState getHoodState() {
        return hoodState;
    }

    public void setState(HoodState state) {
        hoodState = state;
    }

    public double getAngle() {
        return hoodIOInputs.angle;
    }

    public void setAngle(double angle) {
        hoodState = HoodState.ACTIVE;

        requestedAngle = angle;
    }

    public double applyLimits(double angle) {
        double clamped = MathUtil.clamp(angle, Hood.HoodConstants.MIN_ANGLE, Hood.HoodConstants.MAX_ANGLE);

        if (clamped != angle) {
            DriverStation.reportWarning(
                    "Angle requested outside of range [0, 42.5], clamped to %f degrees"
                            .formatted(clamped),
                    false);
        }

        return clamped;
    }
}