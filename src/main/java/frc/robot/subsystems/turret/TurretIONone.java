package frc.robot.subsystems.turret;

/**
 * No-op IO for when the turret isn't physically installed yet (see
 * Constants.HardwarePresence.kTurretInstalled). Never constructs a TalonFX, so it generates zero
 * CAN traffic -- inputs stay at their Java defaults (angle/velocity/etc. 0, turretMotorConnected
 * false), which correctly reports "not connected" since the hardware genuinely isn't there.
 */
public class TurretIONone implements TurretIO {
}
