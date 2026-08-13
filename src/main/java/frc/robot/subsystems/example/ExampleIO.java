package frc.robot.subsystems.example;

public interface ExampleIO {

    default void updateInputs(ExampleInputs inputs) {}

    default void setIntakePivotPosition(double positionDegrees) {}

    default void setIntakeRollerVelocity(double velocityRPM) {}

    default void resetEncoder() {}

    default void stop() {
        setIntakeRollerVelocity(0.0);
    }

    class ExampleInputs {

        public double intakePivotPositionDegrees;
        public double intakePivotAppliedVolts;
        public double intakePivotStatorCurrentAmps;
        public double intakePivotSupplyCurrentAmps;
        public boolean intakePivotMotorConnected;

        public double intakeRollerVelocityRPM;
        public double intakeRollerAppliedVolts;
        public double intakeRollerStatorCurrentAmps;
        public double intakeRollerSupplyCurrentAmps;
        public boolean intakeRollerMotorConnected;
    }
}
