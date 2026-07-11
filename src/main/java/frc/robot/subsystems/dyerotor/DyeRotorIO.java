package frc.robot.subsystems.dyerotor;

public interface DyeRotorIO {
  default void updateInputs(DyeRotorInputs inputs) {
  }

  default void setSpinVelocity(double velocityRPM) {
  }

  default void setIndexVoltage(double volts) {
  }

  default void stop() {
    setSpinVelocity(0.0);
    setIndexVoltage(0.0);
  }

  class DyeRotorInputs {
    // public double spinPositionRotations;
    public double spinVelocityRPM;
    public double spinAppliedVolts;
    public double spinStatorCurrentAmps;
    public double spinSupplyCurrentAmps;
    public boolean spinMotorConnected;

    // public double indexPositionRotations;
    // public double indexVelocityRPM;
    public double indexAppliedVolts;

    public double indexStatorCurrentAmps;
    public double indexSupplyCurrentAmps;

    public boolean indexLeadMotorConnected;
    public boolean indexFollowerMotorConnected;
  }
}