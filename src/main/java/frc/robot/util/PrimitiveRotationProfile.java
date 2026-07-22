package frc.robot.util;

import edu.wpi.first.math.MathUtil;

public class PrimitiveRotationProfile {
    public static class Constraints {
        public final double maxVelocity;
        public final double maxAcceleration;

        public Constraints(double maxVelocity, double maxAcceleration) {
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
        }
    }

    private final Constraints m_constraints;
    private final double m_defaultDt;
    private final double m_maxDt;

    private double m_positionRadians;
    private double m_velocityRadiansPerSecond;
    private double m_lastTimestamp;

    public PrimitiveRotationProfile(Constraints constraints, double defaultDt, double maxDt) {
        m_constraints = constraints;
        m_defaultDt = defaultDt;
        m_maxDt = maxDt;
    }

    public void reset(double positionRadians, double velocityRadiansPerSecond, double timestamp) {
        double maxVelocity = maxVelocity();
        m_positionRadians = positionRadians;
        m_velocityRadiansPerSecond = MathUtil.clamp(
                velocityRadiansPerSecond,
                -maxVelocity,
                maxVelocity);
        m_lastTimestamp = timestamp;
    }

    public void update(double targetRadians, double timestamp) {
        double dt = timestamp - m_lastTimestamp;
        m_lastTimestamp = timestamp;

        if (dt <= 0) {
            dt = m_defaultDt;
        }
        dt = MathUtil.clamp(dt, m_defaultDt, m_maxDt);

        double goalRadians = m_positionRadians
                + MathUtil.inputModulus(targetRadians - m_positionRadians, -Math.PI, Math.PI);
        double error = goalRadians - m_positionRadians;
        double desiredVelocity = calculateDesiredVelocity(error);

        double maxAcceleration = maxAcceleration();
        m_velocityRadiansPerSecond = MathUtil.clamp(
                desiredVelocity,
                m_velocityRadiansPerSecond - maxAcceleration * dt,
                m_velocityRadiansPerSecond + maxAcceleration * dt);
        m_positionRadians += m_velocityRadiansPerSecond * dt;

        if (Math.signum(goalRadians - m_positionRadians) != Math.signum(error)) {
            m_positionRadians = goalRadians;
            m_velocityRadiansPerSecond = 0;
        }
    }

    public double positionRadians() {
        return m_positionRadians;
    }

    public double velocityRadiansPerSecond() {
        return m_velocityRadiansPerSecond;
    }

    public double maxVelocity() {
        return Math.max(0, m_constraints.maxVelocity);
    }

    private double maxAcceleration() {
        return Math.max(0, m_constraints.maxAcceleration);
    }

    private double calculateDesiredVelocity(double error) {
        double maxVelocity = maxVelocity();
        double maxAcceleration = maxAcceleration();

        if (Math.abs(error) <= 1e-6 || maxVelocity <= 0 || maxAcceleration <= 0) {
            return 0;
        }

        double maxVelocityForStopping = Math.sqrt(2 * maxAcceleration * Math.abs(error));
        return Math.copySign(Math.min(maxVelocity, maxVelocityForStopping), error);
    }
}
