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
     * Per-mechanism switches for whether the hardware is actually bolted to the robot right now.
     * A mechanism not installed falls back to its sim IO even on a real (non-simulated) robot, so
     * its "hardware" is a software model instead of CAN calls to a device that isn't there --
     * this is what stops the constant "CAN frame not received/too-stale" spam and the throttled
     * limelight-turret disconnect reports while the robot is only partially built.
     * <p>
     * Flip these to true as each mechanism gets physically wired up.
     */
    public static class HardwarePresence {
        public static final boolean kIntakeInstalled = false;
        public static final boolean kHoodInstalled = false;
        public static final boolean kFlywheelInstalled = false;
        public static final boolean kTurretInstalled = false;
        // DyeRotor and the drivebase are always physically present -- no flag needed.

        // The CANrange sensor is mounted on Intake; the AprilTag camera is mounted on Turret.
        // Neither exists independently of its host mechanism, so they share that flag.
        public static final boolean kCanRangeInstalled = kIntakeInstalled;
        public static final boolean kTurretCameraInstalled = kTurretInstalled;
    }
}
