package frc.robot.util.currentlimit;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Automatically reduces motor current limits based on the state of other subsystems, e.g.
 * lowering intake and drivetrain current limits while the flywheel is shooting so more of
 * the robot's current budget is available where it matters.
 *
 * <p>Register the motors this manager is allowed to throttle with {@link #registerTarget},
 * then describe when they should be throttled with {@link #addRule}. Every loop the rule
 * conditions are re-checked (cheap, no CAN traffic), and a target's current limit is only
 * re-sent to its motor(s) when the resolved limit actually changes, and never more often
 * than {@code minReapplyIntervalSeconds}. That keeps a flickering condition from flooding
 * the CAN bus.
 *
 * <p>The actual hardware push (a blocking CTRE config-apply call, see each target's
 * {@code applyLimit} consumer) is dispatched onto a single dedicated background thread rather
 * than run inline from {@link #periodic()}. Config-apply calls block waiting on a CAN response
 * for up to their configured timeout, and a target motor that isn't actually present on the
 * currently deployed robot will never respond - running that call from periodic() would stall
 * the whole command scheduler loop for the timeout duration every time the rule fires. Applying
 * asynchronously means a slow or missing target can never cause a loop overrun, at the cost of
 * the new limit taking effect slightly later (at most a couple hundred milliseconds) than the
 * rule that requested it - an acceptable tradeoff for something as non-time-critical as a
 * current limit.
 */
public class CurrentLimitManager extends SubsystemBase {

    private static final double kDefaultMinReapplyIntervalSeconds = 0.1;

    private static final class Target {
        final CurrentLimit nominal;
        final Consumer<CurrentLimit> apply;
        CurrentLimit lastApplied;
        double lastAppliedTimestamp = -Double.MAX_VALUE;
        // False until this manager has pushed a limit to hardware at least once. Without this,
        // lastApplied starting at nominal made the first periodic() a no-op (want == lastApplied),
        // so a target whose subsystem never configured it - e.g. a lead/follower pair where only
        // the lead was configured - stayed unlimited until the first rule fired and released.
        boolean everApplied = false;

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
    // reused every call instead of being reallocated at 50Hz.
    // Must be cleared at the start of periodic() before use.
    private final Map<String, CurrentLimit> desiredScratch = new LinkedHashMap<>();
    private boolean enabled = true;

    // Runs target.apply.accept(...) off the main thread; see the class javadoc for why. A single
    // thread is enough - applies are already throttled to one per target per
    // minReapplyIntervalSeconds, and there's no reason to reorder applies relative to each other.
    private final ExecutorService applyExecutor = Executors.newSingleThreadExecutor(runnable -> {
        Thread thread = new Thread(runnable, "CurrentLimitManager-apply");
        thread.setDaemon(true);
        return thread;
    });

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
        Map<String, CurrentLimit> desired = desiredScratch;
        desired.clear();
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
            String name = entry.getKey();
            Target target = entry.getValue();
            // A rule's limit is resolved against the nominal so a rule that only caps one axis
            // doesn't hand the consumer a sentinel for the other - a config-group apply would
            // otherwise write it to hardware. See CurrentLimit's class docs. With no rule active
            // the nominal is already fully resolved, so that path allocates nothing.
            CurrentLimit ruled = desired.get(name);
            CurrentLimit want = (ruled == null) ? target.nominal : ruled.resolvedAgainst(target.nominal);
            boolean dueForReapply = (now - target.lastAppliedTimestamp) >= minReapplyIntervalSeconds;
            if ((!target.everApplied || !want.equals(target.lastApplied)) && dueForReapply) {
                // Bookkeeping updates immediately so reapply throttling/change-detection stays
                // correct even though the hardware push below completes moments later; see
                // dispatchApply().
                target.lastApplied = want;
                target.lastAppliedTimestamp = now;
                target.everApplied = true;
                dispatchApply(name, target.apply, want);
            }
        }

        // Sim-only for debug
        if (RobotBase.isSimulation()) {
            for (Map.Entry<String, Target> entry : targets.entrySet()) {
                CurrentLimit applied = entry.getValue().lastApplied;
                System.out.printf(
                        "[CurrentLimitManager] %s: stator=%s supply=%s%n",
                        entry.getKey(),
                        formatAxis(applied.statorCurrentLimitAmps()),
                        formatAxis(applied.supplyCurrentLimitAmps()));
            }
        }
    }

    private static String formatAxis(double amps) {
        return amps <= 0 ? "n/a (unmanaged)" : String.format("%.1fA", amps);
    }

    // Runs applyLimit off the main thread (see class javadoc). Exceptions are caught rather than
    // left to the executor's default handler: an uncaught exception would otherwise only surface
    // as a silent thread death, and the next dispatchApply() call would just spin up a new worker
    // thread with no indication anything had gone wrong.
    private void dispatchApply(String name, Consumer<CurrentLimit> applyLimit, CurrentLimit want) {
        applyExecutor.execute(() -> {
            try {
                applyLimit.accept(want);
            } catch (RuntimeException e) {
                DriverStation.reportError(
                        "CurrentLimitManager: failed to apply " + want + " to " + name + ": " + e, e.getStackTrace());
            }
        });
    }
}
