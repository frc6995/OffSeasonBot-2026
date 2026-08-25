package frc.robot.subsystems.hood;

import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotVisualizer;
import frc.robot.subsystems.hood.HoodIO.HoodIOInputs;
import frc.robot.util.ShotController.ShooterTargetData;

public class Hood extends SubsystemBase {
    public static class HoodConstants {
        public static final int kCANID = 44; // Should be right with doc

        public static final double[][] kAngleData = {
                // Distance (Meters), Angle(Degrees)
                { 1, 12.5 },
                { 2.5, 20 },
                { 4, 26 },
                { 5, 30},
                { 6, 40}
        };

        // distance from the POI.PASSING_WALL line
        public static final double[][] kPassingAngleData = {
                { 4, 15 },
                { 6.5, 25 },
                { 8, 30 },
                { 10, 40}

 
        };

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

    private HoodIO io;
    private HoodIOInputs inputs = new HoodIOInputs();

    private final MechanismLigament2d hoodLigament = new MechanismLigament2d("hood", Units.inchesToMeters(4), 0, 6,
            new Color8Bit(52, 137, 235));

    private double requestedAngle;

    private HoodState hoodState = HoodState.DISABLED;

    private final Supplier<ShooterTargetData> targetData;


    public Hood(HoodIO io, Supplier<ShooterTargetData> shotData) {
        this.io = io;
        this.targetData = shotData;
        RobotVisualizer.addHood(hoodLigament);
    }
    

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        switch (hoodState) {
            case DISABLED:
                io.disable();
                break;
            case ACTIVE:

                double clampedAngle = applyLimits(targetData.get().hoodAngleDeg());

                io.setAngle(clampedAngle);

        }
    }

    @Override
    public void simulationPeriodic() {
        hoodLigament.setAngle(getAngle());
        RobotVisualizer.updateHood(Units.degreesToRadians(getAngle()));
    }

    public void setState(HoodState state) {
        hoodState = state;
    }

    public void requestActive() {
        setState(HoodState.ACTIVE);
    }

    public void requestDisable() {
        setState(HoodState.DISABLED);
    }

    public void resetEncoder() {
        io.resetEncoder();
    }

    public void setAngle(double angle) {
        hoodState = HoodState.ACTIVE;

        requestedAngle = angle;
    }

    public double applyLimits(double angle) {
        double clamped = MathUtil.clamp(angle, Hood.HoodConstants.MIN_ANGLE, Hood.HoodConstants.MAX_ANGLE);

        return clamped;
    }

    @Logged(name = "State", importance = Importance.CRITICAL)
    public HoodState getState() {
        return hoodState;
    }
    
    @Logged(name = "Connected", importance = Importance.CRITICAL)
    public boolean isConnected() {
        return inputs.hoodMotorConnected;
    }

    @Logged(name = "Angle", importance = Importance.INFO)
    public double getAngle() {
        return inputs.angle;
    }

    @Logged(name = "Setpoint", importance = Importance.INFO)
    public double getRequestedAngle() {
        return requestedAngle;
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
