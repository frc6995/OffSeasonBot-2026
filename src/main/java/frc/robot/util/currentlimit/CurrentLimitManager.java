package frc.robot.util.currentlimit;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Automatically reduces motor current limits based on the state of other subsystems, e.g.
 * lowering intake and drivetrain current limits while the flywheel is shooting so more of
 * the robot's current budget is available where it matters.
 *
 * <p>Register the motors this manager is allowed to throttle with {@link #registerTarget},
 * then describe when they should be throttled with {@link #addRule}. Every loop the rule
 * conditions are re-checked (cheap, no CAN traffic), but a target's current limit is only
 * re-sent to its motor(s) when the resolved limit actually changes, and never more often
 * than {@code minReapplyIntervalSeconds}. That keeps a flickering condition from flooding
 * the CAN bus or blocking the main loop.
 */
public class CurrentLimitManager extends SubsystemBase {

    private static final double kDefaultMinReapplyIntervalSeconds = 0.1;

    private static final class Target {
        final CurrentLimit nominal;
        final Consumer<CurrentLimit> apply;
        CurrentLimit lastApplied;
        double lastAppliedTimestamp = -Double.MAX_VALUE;

        Target(CurrentLimit nominal, Consumer<CurrentLimit> apply) {
            this.nominal = nominal;
            this.apply = apply;
            this.lastApplied = nominal;
        }
    }

    private static final class Rule {
        final BooleanSupplier condition;
        final Map<String, CurrentLimit> targetLimits;

        Rule(BooleanSupplier condition, Map<String, CurrentLimit> targetLimits) {
            this.condition = condition;
            this.targetLimits = targetLimits;
        }
    }

    private final double minReapplyIntervalSeconds;
    private final Map<String, Target> targets = new LinkedHashMap<>();
    private final List<Rule> rules = new ArrayList<>();
    private boolean enabled = true;

    public CurrentLimitManager() {
        this(kDefaultMinReapplyIntervalSeconds);
    }

    public CurrentLimitManager(double minReapplyIntervalSeconds) {
        this.minReapplyIntervalSeconds = minReapplyIntervalSeconds;
    }

    /**
     * Registers a motor (or group of motors moving together, e.g. a lead/follower pair) that
     * this manager is allowed to current-limit. {@code applyLimit} is however the caller wants
     * to actually push a limit to hardware (see subsystem-level setters like
     * {@code Intake.setRollerCurrentLimit}), keeping CAN/TalonFX details out of this class.
     */
    public void registerTarget(String name, CurrentLimit nominalLimit, Consumer<CurrentLimit> applyLimit) {
        targets.put(name, new Target(nominalLimit, applyLimit));
    }

    /**
     * Adds a rule: while {@code condition} is true, each target named in {@code targetLimits}
     * is capped at the paired limit. A target governed by multiple simultaneously-active rules
     * gets the most restrictive (lowest) limit among them.
     */
    public void addRule(BooleanSupplier condition, Map<String, CurrentLimit> targetLimits) {
        for (String name : targetLimits.keySet()) {
            if (!targets.containsKey(name)) {
                throw new IllegalArgumentException("Unknown current limit target: " + name);
            }
        }
        rules.add(new Rule(condition, Map.copyOf(targetLimits)));
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public boolean isEnabled() {
        return enabled;
    }

    @Override
    public void periodic() {
        Map<String, CurrentLimit> desired = new LinkedHashMap<>();
        if (enabled) {
            for (Rule rule : rules) {
                if (!rule.condition.getAsBoolean()) {
                    continue;
                }
                for (Map.Entry<String, CurrentLimit> entry : rule.targetLimits.entrySet()) {
                    desired.merge(entry.getKey(), entry.getValue(), CurrentLimit::mostRestrictive);
                }
            }
        }

        double now = Timer.getFPGATimestamp();
        for (Map.Entry<String, Target> entry : targets.entrySet()) {
            Target target = entry.getValue();
            CurrentLimit want = desired.getOrDefault(entry.getKey(), target.nominal);
            boolean dueForReapply = (now - target.lastAppliedTimestamp) >= minReapplyIntervalSeconds;
            if (!want.equals(target.lastApplied) && dueForReapply) {
                target.apply.accept(want);
                target.lastApplied = want;
                target.lastAppliedTimestamp = now;
            }
        }
    }
}
