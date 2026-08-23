package frc.robot.subsystems.intake;

/**
 * No-op intake IO, used when the intake isn't installed on the robot.
 * See {@link frc.robot.subsystems.flywheel.FlywheelIONone} for why this exists.
 *
 * <p>Note the inherited no-op {@code setRollerCurrentLimits}/{@code setKickerCurrentLimits}:
 * even if the intake were left registered with the CurrentLimitManager, this IO would make
 * those applies free rather than a blocking timeout plus two error reports.
 */
public class IntakeIONone implements IntakeIO {}
