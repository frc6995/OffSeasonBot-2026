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

    /*
     * Motor counts per mechanism. Roller and extension are lead/follower pairs whose followers
     * previously had no current signal at all - only getMotorVoltage() - so half of each pair's
     * draw was missing from the power budget.
     */
    int kRollerMotorCount = 2;
    int kExtensionMotorCount = 2;
    int kKickerMotorCount = 1;

    class IntakeInputs {
        public double rollerAppliedVolts;
        public double rollerVelocityRPM;
        /** Lead motor only; see {@link #rollerMotorSupplyCurrentAmps} for the pair. */
        public double rollerStatorCurrentAmps;
        /** Lead motor only; see {@link #rollerMotorSupplyCurrentAmps} for the pair. */
        public double rollerSupplyCurrentAmps;
        /** Per-motor supply current, indexed [lead, follower]. */
        public double[] rollerMotorSupplyCurrentAmps = new double[kRollerMotorCount];
        public double[] rollerMotorStatorCurrentAmps = new double[kRollerMotorCount];
        public boolean rollerLeadMotorConnected;
        public boolean rollerFollowerMotorConnected;

        public double kickerVelocityRPM;
        public double kickerAppliedVolts;
        public double kickerStatorCurrentAmps;
        public double kickerSupplyCurrentAmps;
        public boolean kickerMotorConnected;

        public double extensionPositionMeters;
        public double extensionAppliedVolts;
        /** Lead motor only; see {@link #extensionMotorSupplyCurrentAmps} for the pair. */
        public double extensionStatorCurrentAmps;
        /** Lead motor only; see {@link #extensionMotorSupplyCurrentAmps} for the pair. */
        public double extensionSupplyCurrentAmps;
        /** Per-motor supply current, indexed [lead, follower]. */
        public double[] extensionMotorSupplyCurrentAmps = new double[kExtensionMotorCount];
        public double[] extensionMotorStatorCurrentAmps = new double[kExtensionMotorCount];
        public boolean extensionLeadMotorConnected;
        public boolean extensionFollowerMotorConnected;
    }
}
