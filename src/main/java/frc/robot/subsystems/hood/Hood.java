package frc.robot.subsystems.hood;

import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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

        public static final double kSafeShotAngle = 20.0;

    }

    public enum HoodState {
        DISABLED,
        ACTIVE,
        SAFE_SHOT
    }

    private HoodIO io;
    private HoodIOInputs inputs = new HoodIOInputs();

    private final MechanismLigament2d hoodLigament = new MechanismLigament2d("hood", Units.inchesToMeters(4), 0, 6,
            new Color8Bit(52, 137, 235));

    // The angle actually sent to the IO this loop, for telemetry (DISABLED leaves this at its last
    // value). Mirrors Turret's commandedAngle.
    private double commandedAngle;

    private HoodState hoodState = HoodState.DISABLED;

    private final Supplier<ShooterTargetData> targetData;


    public Hood(HoodIO io, Supplier<ShooterTargetData> shotData) {
        this.io = io;
        this.targetData = shotData;
        RobotVisualizer.addHood(hoodLigament);
    }
    

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            // An ACTIVE request must never survive a disable -- see Flywheel.periodic() for the
            // full mechanism. Without this the hood drives to its shot angle the instant the robot
            // is re-enabled, with nobody touching the controller.
            setState(HoodState.DISABLED);
        }

        io.updateInputs(inputs);

        switch (hoodState) {
            case DISABLED:
                io.disable();
                break;
            case ACTIVE:
                // Store what was actually sent, so "Setpoint" reflects the live control path.
                // requestedAngle used to be written only by setAngle(), which nothing in the live
                // path calls, so the logged setpoint read a flat zero all match.
                commandedAngle = applyLimits(targetData.get().hoodAngleDeg());
                io.setAngle(commandedAngle);
                break;
            case SAFE_SHOT:
                commandedAngle = applyLimits(HoodConstants.kSafeShotAngle);
                io.setAngle(commandedAngle);
                break;
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

    // setAngle(double) used to live here. It set requestedAngle and flipped the state to ACTIVE,
    // but periodic()'s ACTIVE branch recomputes the angle from targetData every loop and never
    // read requestedAngle -- so the value was discarded on the next tick and only the state change
    // took effect. It had no callers. Removed rather than left as a trap; the Hood has no MANUAL
    // state to make it meaningful the way Turret.setAngle has.

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
