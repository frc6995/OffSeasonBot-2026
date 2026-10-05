package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.wpilibj.DriverStation;

public final class CtreUtil {
    private CtreUtil() {}

    /**
     * Rate at which current and supply voltage signals are published by the motors, in Hz.
     *
     * <p>Set explicitly rather than left to Phoenix's defaults, which vary by signal and by bus
     * type and are not guaranteed to be fast enough for this. Brownout work needs a rate that
     * resolves the event: a voltage sag lasts a couple of hundred milliseconds, so a signal
     * published even a handful of times per second can miss one entirely between two samples.
     * This is the rate the power dashboard in {@code tools/power_analysis} assumes.
     *
     * <p>Verify with {@code getAppliedUpdateFrequency()} on a signal, or read the effective rate
     * off a log - the dashboard reports it per motor and warns when one is too slow.
     *
     * <p>Every motor on this robot is on a CANivore ({@link frc.robot.Constants.CANBuses}), which
     * has the headroom for it - roughly 22 motors x 50 Hz of extra status frames, and Phoenix
     * packs supply current, stator current, and torque current into a single frame so raising all
     * three costs one frame per motor. This would not be safe on the roboRIO's native CAN bus.
     *
     * <p>The hoot log records each signal at exactly this rate, and costs the robot loop nothing
     * either way, so raising it is purely a CAN bandwidth question (check bus utilization in
     * Tuner X). 20 Hz resolves a brownout event.
     */
    public static final double kCurrentSignalFrequencyHz = 20.0;

    /**
     * Rate for the signals the IO layers actually close the loop on - position, velocity and
     * applied voltage - in Hz. This is already the CAN FD default, so requesting it changes
     * nothing by itself.
     *
     * <p>It has to be requested explicitly anyway, because
     * {@link com.ctre.phoenix6.hardware.ParentDevice#optimizeBusUtilizationForAll} slows every
     * signal that was NOT given a frequency down to 4 Hz. Miss one and it drops silently - no
     * error, just a stale reading.
     *
     * <p>A constant rather than a literal at five call sites; the calls themselves are stock
     * {@link BaseStatusSignal#setUpdateFrequencyForAll} on purpose. Intake uses seperate constant.
     */
    public static final double kMechanismSignalFrequencyHz = 100.0;

    /**
     * Publishes the given current signals at {@link #kCurrentSignalFrequencyHz}. Call once from an
     * IO layer's constructor with every current signal it reads; see that constant for why the
     * defaults are not relied on.
     */
    public static void setCurrentSignalFrequency(BaseStatusSignal... signals) {
        reportIfNotOk(
                "set current signal update frequency",
                BaseStatusSignal.setUpdateFrequencyForAll(kCurrentSignalFrequencyHz, signals));
    }

    /**
     * Publishes each motor's supply current and supply voltage at {@link #kCurrentSignalFrequencyHz}
     * so the hoot log captures them for the power dashboard. Call once from an IO layer's
     * constructor with every motor it owns, before any {@code optimizeBusUtilization} call.
     *
     * <p>Nothing in robot code reads these signals. The hoot logger records them as they arrive on
     * the bus, in its own thread, so there is deliberately no per-loop refresh anywhere - that is
     * what keeps power logging off the loop-time budget. Without this call the bus optimizer would
     * drop both to 4 Hz, too slow to see a brownout.
     */
    public static void setPowerSignalFrequency(TalonFX... motors) {
        BaseStatusSignal[] signals = new BaseStatusSignal[motors.length * 2];
        for (int i = 0; i < motors.length; i++) {
            signals[2 * i] = motors[i].getSupplyCurrent(false);
            signals[2 * i + 1] = motors[i].getSupplyVoltage(false);
        }
        reportIfNotOk(
                "set power signal update frequency",
                BaseStatusSignal.setUpdateFrequencyForAll(kCurrentSignalFrequencyHz, signals));
    }

    public static void configureKrakenX60Sim(
            TalonFXSimState simState,
            ChassisReference chassisReference) {
        configureKrakenSim(simState, chassisReference, MotorType.KrakenX60);
    }

    public static void configureKrakenX44Sim(
            TalonFXSimState simState,
            ChassisReference chassisReference) {
        configureKrakenSim(simState, chassisReference, MotorType.KrakenX44);
    }

    private static void configureKrakenSim(
            TalonFXSimState simState,
            ChassisReference chassisReference,
            MotorType motorType) {
        simState.Orientation = chassisReference;
        reportIfNotOk("sim set motor type", simState.setMotorType(motorType));
    }

    public static void reportIfNotOk(String action, StatusCode statusCode) {
        if (!statusCode.isOK()) {
            DriverStation.reportWarning(
                    "CTRE " + action + " returned " + statusCode.getName() + ": "
                            + statusCode.getDescription(),
                    false);
        }
    }
}

