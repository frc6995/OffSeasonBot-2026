package frc.robot.subsystems.intake;

/**
 * No-op IO for when the intake isn't physically installed yet (see
 * Constants.HardwarePresence.kIntakeInstalled). Never constructs a TalonFX, so it generates zero
 * CAN traffic -- inputs stay at their Java defaults (velocities/positions 0, all *Connected flags
 * false), which correctly reports "not connected" since the hardware genuinely isn't there.
 */
public class IntakeIONone implements IntakeIO {
}
