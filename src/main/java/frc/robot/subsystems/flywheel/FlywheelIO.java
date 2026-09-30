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
        /** Lead motor only. For the whole flywheel see {@link #motorSupplyCurrentAmps}. */
        public double supplyCurrentAmps;

        /*
         * Per-motor current, indexed [lead, follower1, follower2, follower3].
         */
        public double[] motorStatorCurrentAmps = new double[kMotorCount];
        public double[] motorSupplyCurrentAmps = new double[kMotorCount];
    }

}
