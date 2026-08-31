package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

/**
 * Rate-limits how often an IO layer re-reads its devices' connection state.
 *
 * <p>{@code ParentDevice.isConnected()} is not a field read. Phoenix implements it as
 * {@code _compliancy.refresh(false).getTimestamp().getLatency() <= maxLatency} -- a JNI status
 * signal refresh on every call. The mechanism IO layers together make fourteen of those per loop
 * (Intake 5, Flywheel 4, DyeRotor 3, Turret 1, Hood 1), which at 50 Hz is 700 refreshes a second.
 *
 * <p>That polling is heavily oversampled: the Version signal it reads updates at 4 Hz by default, so
 * asking fifty times a second returns the same cached answer twelve times over. Polling at 4 Hz
 * gives identical information for about a twelfth of the cost.
 *
 * <p>Usage -- keep the {@code inputs.*Connected} assignments inside the guard. Those fields persist
 * between loops, so they simply hold their last value until the next poll:
 *
 * <pre>{@code
 * private final ConnectionPoll connectionPoll = new ConnectionPoll();
 *
 * public void updateInputs(FooInputs inputs) {
 *     BaseStatusSignal.refreshAll(...);   // still every loop -- this is the real data
 *     ...
 *     if (connectionPoll.due()) {
 *         inputs.motorConnected = m_motor.isConnected();
 *     }
 * }
 * }</pre>
 *
 * <p>The first {@link #due()} call always returns true, so inputs are populated on loop one rather
 * than reading a default {@code false} for the first quarter second.
 *
 * <p>Not thread-safe. Each instance is owned by one IO object and only touched from that object's
 * {@code updateInputs}, which the scheduler calls on the main loop.
 */
public final class ConnectionPoll {
    /**
     * Matches the Version status signal's own 4 Hz default update rate -- polling faster cannot
     * surface a disconnect any sooner, it just repeats work.
     */
    public static final double kDefaultPeriodSeconds = 0.25;

    private final double periodSeconds;
    private double lastPollTimestamp = Double.NEGATIVE_INFINITY;

    public ConnectionPoll() {
        this(kDefaultPeriodSeconds);
    }

    public ConnectionPoll(double periodSeconds) {
        this.periodSeconds = periodSeconds;
    }

    /** @return true at most once per period, and always on the first call. */
    public boolean due() {
        double now = Timer.getFPGATimestamp();
        if (now - lastPollTimestamp < periodSeconds) {
            return false;
        }
        lastPollTimestamp = now;
        return true;
    }
}
