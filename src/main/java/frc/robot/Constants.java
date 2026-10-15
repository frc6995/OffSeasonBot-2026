package frc.robot;

import com.ctre.phoenix6.CANBus;

public class Constants {
    public static class CANBuses {
        public static final CANBus LowerBus = new CANBus("LowerBus", "./logs/lowbus.hoot");
        public static final CANBus UpperBus = new CANBus("UpperBus", "./logs/upbus.hoot");
    }

    // Times per motor that new configs are reapplied used by dynamic current
    // limiting
    public static final double kDynamicConfigTimeoutSeconds = 0.05;
}
