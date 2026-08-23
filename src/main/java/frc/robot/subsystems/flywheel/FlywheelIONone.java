package frc.robot.subsystems.flywheel;

/**
 * No-op flywheel IO, used when the flywheel isn't installed on the robot
 * (see {@link frc.robot.Constants.Hardware}). Constructing no TalonFX at all is the point:
 * it keeps Phoenix from emitting an error report per setControl/refresh against motors that
 * aren't on the CAN bus. Inputs stay at their defaults, so {@code Connected} logs false.
 */
public class FlywheelIONone implements FlywheelIO {}
