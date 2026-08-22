package frc.robot.subsystems.flywheel;

import edu.wpi.first.epilogue.Logged;

public interface FlywheelIO {
    default void updateInputs(FlywheelInputs inputs) {}
    default void setVelocityRPM(double velocityRPM) {}
    default void stop() {}

    public class FlywheelInputs {
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
