package frc.robot.util;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;

/**
 * Central place to declare "while some robot-wide condition holds, cap this
 * motor group's torque current below its normal limit" rules, so cross-
 * subsystem current budgeting doesn't get scattered across subsystem
 * periodic() methods.
 *
 * <p>Example: reduce the intake roller/kicker and drivetrain drive current
 * while the flywheel is spun up to shoot, so the shooter gets priority on the
 * battery/breaker budget.
 * <pre>{@code
 * coordinator.limitWhile(this::isShooting, m_intake.rollerCurrentLimit(), 20.0);
 * coordinator.limitWhile(this::isShooting, m_intake.kickerCurrentLimit(), 20.0);
 * coordinator.limitWhile(this::isShooting, drivetrain.driveCurrentLimit(), 40.0);
 * }</pre>
 * then call {@link #update()} once per scheduler cycle (e.g. from
 * Superstructure.periodic()).
 *
 * <p>Adding a new rule never risks a loop overrun: {@link #update()} just
 * evaluates cheap in-memory conditions and pushes a double to each mechanism.
 * The actual CAN config frame is only sent when a mechanism's effective limit
 * changes - see {@link TorqueCurrentLimiter}, which every {@link
 * LimitedMechanism}'s setter is expected to be backed by.
 */
public class CurrentLimitCoordinator {

    /**
     * One torque-limited motor group: a human-readable, unique name (rules
     * targeting the same mechanism are grouped by this name, not by object
     * identity - so it's fine to build a fresh LimitedMechanism per call, e.g.
     * from a subsystem's own factory method), its normal/unrestricted peak
     * torque current, and the setter that actually pushes a new peak torque
     * current (amps) down to that motor group's IO layer.
     */
    public record LimitedMechanism(String name, double normalPeakTorqueCurrentAmps,
            DoubleConsumer setPeakTorqueCurrentLimitAmps) {
    }

    private record LimitRule(BooleanSupplier condition, String mechanismName,
            double reducedPeakTorqueCurrentAmps) {
    }

    private final List<LimitRule> rules = new ArrayList<>();
    // Keyed by mechanism name (not the LimitedMechanism instance) so multiple
    // rules targeting "the same" mechanism combine correctly even if each was
    // built from a separate LimitedMechanism record.
    private final Map<String, LimitedMechanism> mechanismsByName = new LinkedHashMap<>();

    /**
     * Registers a rule: while {@code condition} is true, {@code mechanism} is
     * capped to at most {@code reducedPeakTorqueCurrentAmps} (amps). Multiple
     * rules may target the same mechanism (matched by name) - the tightest
     * (lowest) active cap wins. When no rule targeting a mechanism is active,
     * it's left at its {@code normalPeakTorqueCurrentAmps}.
     */
    public void limitWhile(BooleanSupplier condition, LimitedMechanism mechanism,
            double reducedPeakTorqueCurrentAmps) {
        mechanismsByName.putIfAbsent(mechanism.name(), mechanism);
        rules.add(new LimitRule(condition, mechanism.name(), reducedPeakTorqueCurrentAmps));
    }

    /**
     * Re-evaluates every rule and pushes the tightest active cap (or the
     * mechanism's normal cap if no rule targeting it is active) to each
     * registered mechanism. Call once per scheduler cycle.
     */
    public void update() {
        for (LimitedMechanism mechanism : mechanismsByName.values()) {
            double limitAmps = mechanism.normalPeakTorqueCurrentAmps();
            for (LimitRule rule : rules) {
                if (rule.mechanismName().equals(mechanism.name()) && rule.condition().getAsBoolean()) {
                    limitAmps = Math.min(limitAmps, rule.reducedPeakTorqueCurrentAmps());
                }
            }
            mechanism.setPeakTorqueCurrentLimitAmps().accept(limitAmps);
        }
    }
}
