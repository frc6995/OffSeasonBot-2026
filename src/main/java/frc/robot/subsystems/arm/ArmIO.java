package frc.robot.subsystems.arm;

public interface ArmIO {

    default void updateInputs(ArmInputs inputs) {}

    default void setElevatorPosition(double positionMeters) {}

    default void setArmPosition(double positionDegrees) {}

    default void setHandVoltage(double volts) {}

    default void resetEncoder() {}

    default void stop() {
        setHandVoltage(0.0);
    }

    class ArmInputs {

        public double elevatorPositionMeters;
        public double elevatorAppliedVolts;
        public double elevatorStatorCurrentAmps;
        public double elevatorSupplyCurrentAmps;
        public boolean elevatorLeadMotorConnected;
        public boolean elevatorFollowerMotorConnected;

        public double armPositionDegrees;
        public double armAppliedVolts;
        public double armStatorCurrentAmps;
        public double armSupplyCurrentAmps;
        public boolean armMotorConnected;

        public double handVelocityRPM;
        public double handAppliedVolts;
        public double handStatorCurrentAmps;
        public double handSupplyCurrentAmps;
        public boolean handMotorConnected;
    }
}
