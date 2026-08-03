package frc.robot.util.currentlimit;

/**
 * A stator/supply current limit pair, in amps.
 *
 * <p>A value {@code <= 0} for either axis means "leave that axis alone," so targets that
 * only manage one of the two limits (e.g. a drivetrain that only caps supply current) can
 * ignore the other without it accidentally winning a {@link #mostRestrictive} comparison.
 */
public record CurrentLimit(double statorCurrentLimitAmps, double supplyCurrentLimitAmps) {

    public static CurrentLimit supplyOnly(double supplyCurrentLimitAmps) {
        return new CurrentLimit(-1.0, supplyCurrentLimitAmps);
    }

    public static CurrentLimit statorOnly(double statorCurrentLimitAmps) {
        return new CurrentLimit(statorCurrentLimitAmps, -1.0);
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
