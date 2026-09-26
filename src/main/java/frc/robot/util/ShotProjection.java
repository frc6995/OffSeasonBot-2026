package frc.robot.util;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Allocation-free shoot-on-the-move solver (virtual goal / pose projection).
 *
 * <p>
 * Call {@link #solve} ONCE per robot loop and have every subsystem read the
 * public result
 * fields. No objects are created per call: all math is on primitives, lookups
 * are binary searches
 * over primitive arrays (no boxing, unlike InterpolatingDoubleTreeMap), and
 * distance uses
 * Math.sqrt rather than Math.hypot, which is several times slower in Java.
 */
public final class ShotProjection {
    public static final class ShotConstants {
        /** The delay applied to the shot as a result of latency or other effects */
        public static final double kShotDelay = 0.02;

        // generated, not imperical rn
        public static final double[][] kTofData = {
                { 1.0, 0.987 }, { 1.5, 0.910 }, { 2.0, 0.970 }, { 2.5, 1.017 }, { 3.0, 1.034 },
                { 3.5, 0.991 }, { 4.0, 0.944 }, { 4.5, 0.995 }, { 5.0, 1.040 }, { 5.5, 1.092 },
                { 6.0, 1.134 }
        };
    }

    /**
     * Piecewise-linear lookup table over primitive arrays. Keys must be sorted
     * ascending.
     */
    public static final class Lut {
        private final double[] x, y;

        public Lut(double[][] data) {
            x = new double[data.length];
            y = new double[data.length];
            for (int i = 0; i < data.length; i++) {
                x[i] = data[i][0];
                y[i] = data[i][1];
            }
        }

        /** Clamps outside the table, like InterpolatingDoubleTreeMap. */
        public double get(double d) {
            int n = x.length;
            if (d <= x[0])
                return y[0];
            if (d >= x[n - 1])
                return y[n - 1];
            int lo = 0, hi = n - 1;
            while (hi - lo > 1) {
                int mid = (lo + hi) >>> 1;
                if (x[mid] <= d)
                    lo = mid;
                else
                    hi = mid;
            }
            double f = (d - x[lo]) / (x[hi] - x[lo]);
            return y[lo] + f * (y[hi] - y[lo]);
        }
    }

    private static final int kMaxIterations = 5;
    private static final double kConvergedMeters = 1e-3;

    private final DoublePublisher m_distancePublisher;

    private final Lut tofLut, hoodLut, rpmLut, passRPMLut,passHoodLut;

    // ---- Results of the last solve(); read these, don't recompute. ----
    /** Distance from the release point to the virtual goal, meters. */
    public double distance;
    /**
     * Field-relative bearing from the release point to the virtual goal, radians.
     */
    public double robotAngleRad;
    public double hoodDeg;
    public double rpm;
    public double tofSec;
    /** Virtual goal and release point, for telemetry. */
    public double virtualGoalX, virtualGoalY, releaseX, releaseY;

    public ShotProjection(double[][] tofData, double[][] hoodData, double[][] rpmData, double[][] passRPMData,
            double[][] passHoodData) {
        tofLut = new Lut(tofData);
        hoodLut = new Lut(hoodData);
        rpmLut = new Lut(rpmData);
        passRPMLut = new Lut(passRPMData);
        passHoodLut = new Lut(passHoodData);

        m_distancePublisher = NetworkTableInstance.getDefault().getTable("ShotProjection").getDoubleTopic("Distance").publish();
    }

    /**
     * @param rx,             ry robot (turret) position, field frame, meters
     * @param vx,             vy robot velocity, FIELD-relative, m/s
     * @param gx,             gy real goal position, field frame, meters
     * @param releaseDelaySec time from this loop until the ball actually leaves the
     *                        shooter
     *                        (loop + pose latency + feeder), kept separate from
     *                        time of flight
     */
    public void solve(double rx, double ry, double rtheta, double vx, double vy,
            double gx, double gy, double releaseDelaySec) {
        // Where the robot will be when the ball leaves.
        double px = rx + vx * releaseDelaySec;
        double py = ry + vy * releaseDelaySec;

        // Start from the real goal, then shift it opposite the robot's velocity by the
        // time of
        // flight. TOF barely changes with distance, so this converges in 2-3 passes.
        double tx = gx, ty = gy;
        double dx = tx - px, dy = ty - py;
        double d = Math.sqrt(dx * dx + dy * dy);
        double t = tofLut.get(d);

        for (int i = 0; i < kMaxIterations; i++) {
            tx = gx - vx * t;
            ty = gy - vy * t;
            dx = tx - px;
            dy = ty - py;
            double nd = Math.sqrt(dx * dx + dy * dy);
            boolean converged = Math.abs(nd - d) < kConvergedMeters;
            d = nd;
            t = tofLut.get(d);
            if (converged)
                break;
        }

        distance = d;
        tofSec = t;
        robotAngleRad = Math.atan2(dy, dx) - rtheta;
        hoodDeg = hoodLut.get(d);
        rpm = rpmLut.get(d);
        virtualGoalX = tx;
        virtualGoalY = ty;
        releaseX = px;
        releaseY = py;

        m_distancePublisher.accept(d);
    }

    /**
     * @param rx, ry       robot position, field frame, meters (ry unused: the passing wall is
     *                     vertical, so only the x-distance to it matters)
     * @param rtheta       robot heading, field frame, radians
     * @param gx           wall x position, field frame, meters
     * @param targetAngleRad field-relative heading to aim at, radians (e.g. POI.PASSING_ANGLE)
     */
    public void solvePassing(double rx, double ry, double rtheta, double gx, double targetAngleRad) {
        double dist = Math.abs(rx - gx);
        robotAngleRad = targetAngleRad - rtheta;
        rpm = passRPMLut.get(dist);
        hoodDeg = passHoodLut.get(dist);
    }
}