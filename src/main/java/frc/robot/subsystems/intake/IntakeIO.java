package frc.robot.subsystems.intake;

import edu.wpi.first.epilogue.Logged;

public interface IntakeIO {
    default void updateInputs(IntakeInputs inputs) {}
    default void setRollerVelocity(double velocityRPM) {}
    default void setKickerVelocity(double velocityRPM) {}
    default void setExtensionPosition(double positionMeters) {}

    default void setRollerCurrentLimits(double statorCurrentLimitAmps, double supplyCurrentLimitAmps) {}

    default void setKickerCurrentLimits(double statorCurrentLimitAmps, double supplyCurrentLimitAmps) {}

    default void resetEncoder() {}

    default void stop() {
        setRollerVelocity(0.0);
        setKickerVelocity(0.0);
    }

    class IntakeInputs {
        public double rollerAppliedVolts;
        public double rollerVelocityRPM;
        public double rollerStatorCurrentAmps;
        public double rollerSupplyCurrentAmps;
        public boolean rollerLeadMotorConnected;
        public boolean rollerFollowerMotorConnected;

        public double kickerVelocityRPM;
        public double kickerAppliedVolts;
        public double kickerStatorCurrentAmps;
        public double kickerSupplyCurrentAmps;
        public boolean kickerMotorConnected;

        public double extensionPositionMeters;
        public double extensionAppliedVolts;
        public double extensionStatorCurrentAmps;
        public double extensionSupplyCurrentAmps;
        public boolean extensionLeadMotorConnected;
        public boolean extensionFollowerMotorConnected;
    }
}
