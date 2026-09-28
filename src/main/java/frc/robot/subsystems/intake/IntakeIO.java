package frc.robot.subsystems.intake;

import edu.wpi.first.epilogue.Logged;

public interface IntakeIO {
    default void updateInputs(IntakeInputs inputs) {}
    default void setRollerVoltage(double voltage) {}
    default void setKickerVoltage(double voltage) {}
    default void setExtensionPosition(double positionMeters) {}

    default void setRollerCurrentLimits(double statorCurrentLimitAmps, double supplyCurrentLimitAmps) {}

    default void setKickerCurrentLimits(double statorCurrentLimitAmps, double supplyCurrentLimitAmps) {}

    default void resetEncoder() {}

    default void stop() {
        setRollerVoltage(0.0);
        setKickerVoltage(0.0);
    }

    /* Motor counts per mechanism. Roller and extension are lead/follower pairs. */
    int kRollerMotorCount = 2;
    int kExtensionMotorCount = 2;
    int kKickerMotorCount = 1;

    class IntakeInputs {
        public double rollerAppliedVolts;
        /** Lead motor only; see {@link #rollerMotorStatorCurrentAmps} for the pair. */
        public double rollerStatorCurrentAmps;
        /** Per-motor stator current, indexed [lead, follower]. */
        public double[] rollerMotorStatorCurrentAmps = new double[kRollerMotorCount];

        public double kickerAppliedVolts;
        public double kickerStatorCurrentAmps;

        public double extensionPositionMeters;
        public double extensionAppliedVolts;
        /** Lead motor only; see {@link #extensionMotorStatorCurrentAmps} for the pair. */
        public double extensionStatorCurrentAmps;
        /** Per-motor stator current, indexed [lead, follower]. */
        public double[] extensionMotorStatorCurrentAmps = new double[kExtensionMotorCount];
    }
}
