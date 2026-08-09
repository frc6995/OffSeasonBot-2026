package frc.robot.subsystems.flywheel;

public interface FlywheelIO {

    default void updateInputs(FlywheelInputs inputs) {
    }

    default void setVelocityRPM(double velocityRPM) {
    }

    /** Caps the closed-loop torque current (amps) the flywheel is allowed to command. */
    default void setPeakTorqueCurrentLimit(double peakTorqueCurrentAmps) {
    }

    default void stop() {
    }

    class FlywheelInputs {
        public double velocityRPM;
        public double appliedVolts;
        public double statorCurrentAmps;
        public double supplyCurrentAmps;
        public boolean leadMotorConnected;
        public boolean followerMotor1Connected;
        public boolean followerMotor2Connected;
        public boolean followerMotor3Connected;
    }

}
