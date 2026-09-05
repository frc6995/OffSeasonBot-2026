package frc.robot.subsystems.dyerotor;

import edu.wpi.first.epilogue.Logged;

public interface DyeRotorIO {
  default void updateInputs(DyeRotorInputs inputs) {}
  default void setSpinVelocity(double velocityRPM) {}
  default void setIndexVoltage(double volts) {}
  default void setIndexVelocity(double velocityRPM) {}
  default void stop() {}

  /** The indexer is a lead/follower pair; the spin motor is on its own. */
  int kIndexMotorCount = 2;

  public class DyeRotorInputs {
    public double spinPositionRotations;
    public double spinVelocityRPM;
    public double spinAppliedVolts;
    public double spinStatorCurrentAmps;
    public double spinSupplyCurrentAmps;
    public boolean spinMotorConnected;

    public double indexPositionRotations;
    public double indexVelocityRPM;
    public double indexAppliedVolts;
    /** Lead motor only; see {@link #indexMotorSupplyCurrentAmps} for the pair. */
    public double indexStatorCurrentAmps;
    /** Lead motor only; see {@link #indexMotorSupplyCurrentAmps} for the pair. */
    public double indexSupplyCurrentAmps;
    /**
     * Per-motor supply current, indexed [lead, follower]. The follower previously had no current
     * signal at all, so half the indexer's draw was missing from the power budget.
     */
    public double[] indexMotorSupplyCurrentAmps = new double[kIndexMotorCount];
    public double[] indexMotorStatorCurrentAmps = new double[kIndexMotorCount];
    public boolean indexLeadMotorConnected;
    public boolean indexFollowerMotorConnected;
  }
}