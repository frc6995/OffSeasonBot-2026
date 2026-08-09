package frc.robot.subsystems.intake;

public interface IntakeIO {

    default void updateInputs(IntakeInputs inputs) {}

    default void setRollerVelocity(double velocityRPM) {}

    default void setKickerVelocity(double velocityRPM) {}

    default void setExtensionPosition(double positionMeters) {}

    /** Caps the closed-loop torque current (amps) the extension is allowed to command. */
    default void setExtensionTorqueCurrentLimit(double peakTorqueCurrentAmps) {}

    /** Caps the closed-loop torque current (amps) the roller is allowed to command. */
    default void setRollerTorqueCurrentLimit(double peakTorqueCurrentAmps) {}

    /** Caps the closed-loop torque current (amps) the kicker is allowed to command. */
    default void setKickerTorqueCurrentLimit(double peakTorqueCurrentAmps) {}

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
