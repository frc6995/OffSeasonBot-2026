package frc.robot;

import com.ctre.phoenix6.CANBus;

public class Constants {
    public static class CANBuses {
        public static final CANBus LowerBus = new CANBus("LowerBus");
        public static final CANBus UpperBus = new CANBus("UpperBus");
    }

    /**
     * Which mechanisms are physically installed on the robot that's currently assembled.
     *
     * <p>A subsystem whose flag is false gets a no-op IO ({@code ...IONone}) instead of its
     * TalonFX IO, so no {@link com.ctre.phoenix6.hardware.TalonFX} is ever constructed for it.
     * This matters for CPU, not just tidiness: Phoenix reports an error through
     * {@code ErrorReportingJNI} on every {@code setControl} and every {@code refreshAll} that
     * targets a device which isn't on the bus. With a mechanism's motors declared but absent,
     * that's an error report per motor per 20ms loop, forever - roboRIO error reporting does
     * string formatting, takes a global lock, and writes to netconsole and the DS, so it is not
     * cheap. The same goes for runtime config applies (see RobotCurrentLimits): an apply to an
     * absent motor blocks for its full timeout and then reports twice, once from CTRE and once
     * from our own CtreUtil.reportIfNotOk.
     *
     * <p>Flip a flag to true as the mechanism goes on the robot. Simulation ignores these
     * entirely and always uses the sim IOs, so sim behavior is unchanged.
     */
    public static class Hardware {
        public static final boolean kIntakeInstalled = false;
        public static final boolean kHoodInstalled = false;
        public static final boolean kFlywheelInstalled = false;
        public static final boolean kTurretInstalled = false;
        public static final boolean kDyeRotorInstalled = true;
        /** The CANrange wall sensor used by BLine Depot auto (CAN ID 35, UpperBus). */
        public static final boolean kCANRangeInstalled = false;
    }
}
