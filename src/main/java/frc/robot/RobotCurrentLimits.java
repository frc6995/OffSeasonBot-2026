package frc.robot;

import java.util.HashMap;
import java.util.Map;

import frc.robot.Constants.Hardware;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.RobotState;
import frc.robot.subsystems.intake.Intake.IntakeConstants;
import frc.robot.util.currentlimit.CurrentLimit;
import frc.robot.util.currentlimit.CurrentLimitManager;

/**
 * Declares the dynamic current limiting rules for the robot: which motors the
 * {@link CurrentLimitManager} is allowed to throttle, and the conditions under which it should.
 *
 * <p>To add a new rule (new subsystem interaction, new condition, etc.), register any new
 * targets with {@link CurrentLimitManager#registerTarget} and add a
 * {@link CurrentLimitManager#addRule} call below - no changes to the manager itself are needed.
 *
 * <p>Targets are only registered for mechanisms that are actually installed (see
 * {@link Constants.Hardware}). A config apply to a motor that isn't on the CAN bus blocks for its
 * full timeout and then reports the failure twice - once from inside Phoenix's
 * {@code ParentConfigurator}, once from our own {@code CtreUtil.reportIfNotOk} - and that whole
 * burst repeats on every entry to and exit from SCORING/PASSING. Skipping registration keeps the
 * rule down to the motors that can actually answer.
 */
public final class RobotCurrentLimits {
    private RobotCurrentLimits() {}

    // Reduced limits applied to the intake and drivetrain while the robot is scoring or
    // passing (i.e. the flywheel is spun up), freeing up current budget for the flywheel.
    // Tune these to taste.
    private static final CurrentLimit kShootingRollerLimit = new CurrentLimit(40, 20);
    private static final CurrentLimit kShootingKickerLimit = new CurrentLimit(40, 20);
    private static final CurrentLimit kShootingDriveLimit = CurrentLimit.supplyOnly(1);

    public static void configure(
            CurrentLimitManager manager,
            Superstructure superstructure,
            CommandSwerveDrivetrain drivetrain) {

        Map<String, CurrentLimit> shootingLimits = new HashMap<>();

        if (Hardware.kIntakeInstalled) {
            manager.registerTarget(
                    "Intake/Roller",
                    new CurrentLimit(
                            IntakeConstants.kRollerStatorCurrentLimit,
                            IntakeConstants.kRollerSupplyCurrentLimit),
                    superstructure.m_intake::setRollerCurrentLimit);
            shootingLimits.put("Intake/Roller", kShootingRollerLimit);

            manager.registerTarget(
                    "Intake/Kicker",
                    new CurrentLimit(
                            IntakeConstants.kKickerStatorCurrentLimit,
                            IntakeConstants.kKickerSupplyCurrentLimit),
                    superstructure.m_intake::setKickerCurrentLimit);
            shootingLimits.put("Intake/Kicker", kShootingKickerLimit);
        }

        manager.registerTarget(
                "Drivetrain/Drive",
                CurrentLimit.supplyOnly(TunerConstants.kDriveNominalSupplyCurrentLimitAmps),
                limit -> drivetrain.setDriveSupplyCurrentLimit(limit.supplyCurrentLimitAmps()));
        shootingLimits.put("Drivetrain/Drive", kShootingDriveLimit);

        manager.addRule(
                () -> {
                    RobotState state = superstructure.getRobotState();
                    return state == RobotState.SCORING || state == RobotState.PASSING;
                },
                shootingLimits);
    }
}
