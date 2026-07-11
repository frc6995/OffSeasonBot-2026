package frc.robot.subsystems.Intake;

public interface IntakeIO {
    
    default void updateInputs (IntakeInputs inputs) {

    }

    default void setRollerVoltage(double volts) {

    }

    default void setKickVoltage(double volts) {

    }

    default void setExtensionVoltage(double meters) {
        
    }

    default void stop() {

    }


    class IntakeInputs {

        public double rollerAppliedVolts;
        public double rollerStatorCurrentAmps;
        public double rollerSupplyCurrentAmps;
        public boolean rollerLeadMotorConnected;
        public boolean rollerFollowerMotorConnected;

        public double kickAppliedVolts;
        public double kickStatorCurrentAmps;
        public double kickSupplyCurrentAmps;
        public boolean kickMotorConnected;

        public double extensionAppliedVolts;
        public double extensionStatorCurrentAmps;
        public double extensionSupplyCurrentAmps;
        public boolean extensionLeadMotorConnected;
        public boolean extensionFollowerMotorConnected;
    }

    
}
