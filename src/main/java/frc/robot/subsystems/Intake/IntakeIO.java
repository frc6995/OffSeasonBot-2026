package frc.robot.subsystems.Intake;

public interface IntakeIO {

    default void updateInputs(IntakeInputs inputs) {}

    default void setRollerVoltage(double volts) {}

    default void setKickerVoltage(double volts) {}

    default void setExtensionPosition(double positionMeters) {}

    default void resetEncoder() {}

    default void stop() {
        setRollerVoltage(0.0);
        setKickerVoltage(0.0);
    }



    class IntakeInputs {

        public double rollerAppliedVolts;
        public double rollerStatorCurrentAmps;
        public double rollerSupplyCurrentAmps;
        public boolean rollerLeadMotorConnected;
        public boolean rollerFollowerMotorConnected;

        public double kickerAppliedVolts;
        public double kickerStatorCurrentAmps;
        public double kickerSupplyCurrentAmps;
        public boolean kickerMotorConnected;

        public double extensionAppliedVolts;
        public double extensionStatorCurrentAmps;
        public double extensionSupplyCurrentAmps;
        public boolean extensionLeadMotorConnected;
        public boolean extensionFollowerMotorConnected;
    }

    
}
