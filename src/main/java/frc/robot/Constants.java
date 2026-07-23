package frc.robot;

import com.ctre.phoenix6.CANBus;

public class Constants {
    public static class CANBuses {
        public static final CANBus LowerBus = new CANBus("LowerBus", "./logs/lowbus.hoot");
        public static final CANBus UpperBus = new CANBus("UpperBus","./logs/upbus.hoot");
    }
}
