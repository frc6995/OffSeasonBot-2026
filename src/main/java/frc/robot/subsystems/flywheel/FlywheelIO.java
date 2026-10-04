package frc.robot.subsystems.flywheel;

public interface FlywheelIO {
    default void updateInputs(FlywheelInputs inputs) {}
    default void setVelocityRPM(double velocityRPM) {}
    default void stop() {}

    /** Number of motors on the flywheel: one lead plus three followers. */
    public static final int kMotorCount = 4;

    public class FlywheelInputs {
        public double velocityRPM;
        public double appliedVolts;
        /** Lead motor only. For the whole flywheel see {@link #motorStatorCurrentAmps}. */
        public double statorCurrentAmps;

        /*
         * Per-motor current, indexed [lead, follower1, follower2, follower3].
         *
         * Followers run the same setpoint but not the same load, so these cannot be inferred from
         * the lead.
         */
        public double[] motorStatorCurrentAmps = new double[kMotorCount];
    }

}
