package frc.robot.util.currentlimit;

/**
 * A stator/supply current limit pair, in amps.
 *
 * <p>A value {@code <= 0} for either axis means "this limit does not manage that axis," so a
 * rule that only caps supply current can ignore the stator axis without it losing a
 * {@link #mostRestrictive} comparison.
 *
 * <p>Note what that can and cannot mean at the hardware layer. Phoenix's
 * {@code CurrentLimitsConfigs} is a config <em>group</em>: applying it overwrites both axes, so
 * an unmanaged axis cannot be left untouched - something has to be written for it. The manager
 * therefore resolves every unmanaged axis against the target's nominal limit (see
 * {@link #resolvedAgainst}) before handing a limit to a consumer, which is what "leave that axis
 * alone" actually resolves to: keep it at its unthrottled value. An axis that is still unmanaged
 * after that resolution is one no one ever specified, and consumers must disable that limit
 * rather than write the sentinel to hardware.
 */
public record CurrentLimit(double statorCurrentLimitAmps, double supplyCurrentLimitAmps) {

    public static CurrentLimit supplyOnly(double supplyCurrentLimitAmps) {
        return new CurrentLimit(-1.0, supplyCurrentLimitAmps);
    }

    public static CurrentLimit statorOnly(double statorCurrentLimitAmps) {
        return new CurrentLimit(statorCurrentLimitAmps, -1.0);
    }

    /** Whether this limit specifies a stator limit at all. */
    public boolean managesStator() {
        return statorCurrentLimitAmps > 0;
    }

    /** Whether this limit specifies a supply limit at all. */
    public boolean managesSupply() {
        return supplyCurrentLimitAmps > 0;
    }

    /**
     * Returns this limit with any unmanaged axis filled in from {@code fallback}. Used to turn a
     * partial limit (e.g. a {@link #supplyOnly} rule) into a fully-specified one against a
     * target's nominal, so that applying it as a config group doesn't wipe out the axis the rule
     * never meant to touch.
     */
    CurrentLimit resolvedAgainst(CurrentLimit fallback) {
        return new CurrentLimit(
                managesStator() ? statorCurrentLimitAmps : fallback.statorCurrentLimitAmps,
                managesSupply() ? supplyCurrentLimitAmps : fallback.supplyCurrentLimitAmps);
    }

    static CurrentLimit mostRestrictive(CurrentLimit a, CurrentLimit b) {
        return new CurrentLimit(
                mostRestrictiveAxis(a.statorCurrentLimitAmps, b.statorCurrentLimitAmps),
                mostRestrictiveAxis(a.supplyCurrentLimitAmps, b.supplyCurrentLimitAmps));
    }

    private static double mostRestrictiveAxis(double a, double b) {
        if (a <= 0) return b;
        if (b <= 0) return a;
        return Math.min(a, b);
    }
}
