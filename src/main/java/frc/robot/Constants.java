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
}
