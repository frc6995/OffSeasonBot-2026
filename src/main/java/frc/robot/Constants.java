package frc.robot;

import com.ctre.phoenix6.CANBus;

public class Constants {
    public static class CANBuses {
        public static final CANBus LowerBus = new CANBus("LowerBus", "./logs/lowbus.hoot");
        public static final CANBus UpperBus = new CANBus("UpperBus","./logs/upbus.hoot");
    }

    // CTRE config-apply calls reject a timeout of 0 outright (StatusCode.TimeoutCannotBeZero) -
    // they always block waiting for a CAN response, up to this timeout, so this must be a real
    // positive value. That's fine here: CurrentLimitManager dispatches this method off the main
    // thread specifically so this blocking can't cause a loop overrun.
    public static final double kDynamicConfigTimeoutSeconds = 0.05;

    /**
     * Master switch for the per-loop power data collection that feeds {@code tools/power_analysis}.
     *
     * <p>Currently OFF while we chase loop overruns. Turning it off costs, per 20 ms loop: six
     * {@link edu.wpi.first.wpilibj.PowerDistribution} JNI reads (PowerMonitor is not constructed,
     * so it never registers with the scheduler), sixteen swerve supply-current signal reads in
     * {@code CommandSwerveDrivetrain.periodic()}, and two Epilogue logger subtrees.
     *
     * <p>Set back to {@code true} to collect power data again - nothing else needs changing, and
     * no channel names move, so an existing report still reads the same. Note the per-subsystem
     * {@code Supply Current Total} channels are NOT gated by this: those read from inputs the IO
     * layers already refresh in their existing batched calls, so they cost close to nothing.
     * Without this flag on, though, there is no battery voltage or PDP total to interpret them
     * against, so a log taken with it off cannot be analysed.
     */
    public static final boolean kPowerLoggingEnabled = false;
}
