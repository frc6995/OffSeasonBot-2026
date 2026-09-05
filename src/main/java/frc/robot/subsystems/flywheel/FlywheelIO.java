package frc.robot.subsystems.flywheel;

import edu.wpi.first.epilogue.Logged;

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
         *
         * The three followers were previously not read at all, so roughly three quarters of the
         * flywheel's draw was invisible to logging - which made it the single largest blind spot
         * in the robot's power budget. Followers run the same setpoint but not the same load, so
         * these cannot be inferred from the lead.
         */
        public double[] motorStatorCurrentAmps = new double[kMotorCount];
        public double[] motorSupplyCurrentAmps = new double[kMotorCount];

        public boolean leadMotorConnected;
        public boolean followerMotor1Connected;
        public boolean followerMotor2Connected;
        public boolean followerMotor3Connected;
    }

}
