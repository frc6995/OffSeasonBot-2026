package frc.robot.subsystems.flywheel;

/**
 * No-op IO for when the flywheel isn't physically installed yet (see
 * Constants.HardwarePresence.kFlywheelInstalled). Never constructs a TalonFX, so it generates
 * zero CAN traffic -- inputs stay at their Java defaults (velocityRPM/etc. 0, all *Connected
 * flags false), which correctly reports "not connected" since the hardware genuinely isn't there.
 */
public class FlywheelIONone implements FlywheelIO {
}
