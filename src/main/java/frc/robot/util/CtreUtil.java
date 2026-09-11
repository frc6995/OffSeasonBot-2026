package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.wpilibj.DriverStation;

public final class CtreUtil {
    private CtreUtil() {}

    /**
     * Rate at which supply/stator current signals are published by the motors, in Hz.
     *
     * <p>Set explicitly rather than left to Phoenix's defaults, which vary by signal and by bus
     * type and are not guaranteed to be fast enough for this. Brownout work needs a rate that
     * resolves the event: a voltage sag lasts a couple of hundred milliseconds, so a signal
     * published even a handful of times per second can miss one entirely between two samples.
     * This is the rate the offline power analysis in {@code tools/power_analysis} assumes.
     *
     * <p>Verify with {@code getAppliedUpdateFrequency()} on a signal, or read the effective rate
     * off a log - the analysis reports it per channel and warns when one is too slow.
     *
     * <p>Every motor on this robot is on a CANivore ({@link frc.robot.Constants.CANBuses}), i.e.
     * CAN FD, where supply and stator current already default to 100 Hz - the 4 Hz figure people
     * quote is the CAN 2.0 default and does not apply here. So this constant REDUCES those two
     * signals rather than raising them, and is a bus-traffic saving, not a cost.
     *
     * <p>20 Hz still resolves a brownout event (a sag lasts a couple of hundred milliseconds).
     * Note it is below the 50 Hz robot loop, so a given loop may read the same sample twice;
     * that is fine for power analysis and is what keeps the lazy log backend from writing every
     * current topic on every single loop.
     */
    public static final double kCurrentSignalFrequencyHz = 20.0;

    /**
     * Rate for the signals the IO layers actually close the loop on - position, velocity, applied
     * voltage - in Hz. This is the CAN FD default, so setting it changes nothing on its own.
     *
     * <p>It has to be set explicitly anyway, because {@link #optimizeBusUtilization} slows every
     * signal that was NOT given a frequency down to 4 Hz. Without this call, optimizing would
     * quietly drop the hood and turret position feedback to 4 Hz and break their control.
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
     * Publishes the given mechanism signals at {@link #kMechanismSignalFrequencyHz}. Call once
     * from an IO layer's constructor with every position/velocity/voltage signal it reads, before
     * {@link #optimizeBusUtilization} - see that constant for why this is not optional.
     */
    public static void setMechanismSignalFrequency(BaseStatusSignal... signals) {
        reportIfNotOk(
                "set mechanism signal update frequency",
                BaseStatusSignal.setUpdateFrequencyForAll(kMechanismSignalFrequencyHz, signals));
    }

    /**
     * Slows every status frame the given devices publish that this code never reads, down to 4 Hz.
     *
     * <p>On CAN FD a TalonFX defaults to publishing essentially everything at 100 Hz: duty cycle,
     * torque current, device and processor temperature, closed-loop telemetry, fault and
     * sticky-fault frames. The IO layers here read position, velocity, applied voltage and the two
     * currents; every other frame is bus bandwidth and RIO-side decode work spent on data nothing
     * looks at. Phoenix has no way to know that unless it is told.
     *
     * <p>Call this LAST in an IO constructor, after the {@code set*SignalFrequency} calls - not
     * because Phoenix requires that order, but because a signal this code reads and forgets to
     * request is silently dropped to 4 Hz, and keeping the calls adjacent makes that easy to spot.
     *
     * <p>Blocks for up to 100 ms per device, so this is a startup cost only.
     *
     * @param context Short name used in the warning if the call fails, e.g. "Turret".
     */
    public static void optimizeBusUtilization(String context, ParentDevice... devices) {
        reportIfNotOk(
                "optimize bus utilization (" + context + ")",
                ParentDevice.optimizeBusUtilizationForAll(devices));
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

