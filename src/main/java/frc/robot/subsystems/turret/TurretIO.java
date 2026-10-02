package frc.robot.subsystems.turret;
public interface TurretIO {
    public default void setAngle(double angle) {};
    /** @param velocityDegPerSec feedforward: rate the setpoint itself is moving (deg/s). */
    public default void setAngle(double angle, double velocityDegPerSec) { setAngle(angle); }
    public default void resetEncoder() {};
    public default void updateInputs(TurretIOInputs inputs) {};
    public default void disable() {};

    public class TurretIOInputs {
        public double angle;
        /** Turret angular velocity in degrees per second. */
        public double velocity;
        public double appliedVolts;
        public double statorCurrent;
        public double supplyCurrent;
    }

}
