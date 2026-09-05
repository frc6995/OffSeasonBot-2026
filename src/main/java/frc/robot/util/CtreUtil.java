package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.wpilibj.DriverStation;

public final class CtreUtil {
    private CtreUtil() {}

    /**
     * Rate at which supply/stator current signals are published by the motors, in Hz.
     *
     * <p>Phoenix 6 defaults current signals to a much lower rate than the mechanism signals
     * (4 Hz on TalonFX). That is fine for a dashboard readout but useless for brownout work: a
     * voltage sag lasts a couple hundred milliseconds, so at 4 Hz the entire event falls between
     * two samples. This is the rate the offline power analysis in {@code tools/power_analysis}
     * assumes.
     *
     * <p>Every motor on this robot is on a CANivore ({@link frc.robot.Constants.CANBuses}), which
     * has the headroom for it - roughly 22 motors x 50 Hz of extra status frames, and Phoenix
     * packs supply current, stator current, and torque current into a single frame so raising all
     * three costs one frame per motor. This would not be safe on the roboRIO's native CAN bus.
     *
     * <p>If bus utilization (logged as {@code Power/CAN/*}) turns out too high, 20 Hz still
     * resolves a brownout event - drop this constant rather than removing the calls.
     */
    public static final double kCurrentSignalFrequencyHz = 50.0;

    /**
     * Publishes the given current signals at {@link #kCurrentSignalFrequencyHz}. Call once from an
     * IO layer's constructor with every current signal it reads; see that constant for why the
     * defaults are not good enough.
     */
    public static void setCurrentSignalFrequency(BaseStatusSignal... signals) {
        reportIfNotOk(
                "set current signal update frequency",
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

