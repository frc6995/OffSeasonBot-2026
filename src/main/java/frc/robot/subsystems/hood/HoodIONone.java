package frc.robot.subsystems.hood;

/**
 * No-op IO for when the hood isn't physically installed yet (see
 * Constants.HardwarePresence.kHoodInstalled). Never constructs a TalonFX, so it generates zero
 * CAN traffic -- inputs stay at their Java defaults (angle/etc. 0, hoodMotorConnected false),
 * which correctly reports "not connected" since the hardware genuinely isn't there.
 */
public class HoodIONone implements HoodIO {
}
