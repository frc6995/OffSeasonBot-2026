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

    public double indexPositionRotations;
    public double indexVelocityRPM;
    public double indexAppliedVolts;
    /** Lead motor only; see {@link #indexMotorStatorCurrentAmps} for the pair. */
    public double indexStatorCurrentAmps;
    /** Per-motor stator current, indexed [lead, follower]. */
    public double[] indexMotorStatorCurrentAmps = new double[kIndexMotorCount];
  }
}