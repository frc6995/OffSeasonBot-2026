package frc.robot.subsystems.dyerotor;

public interface DyeRotorIO {
  default void updateInputs(DyeRotorInputs inputs) {
  }

  default void setSpinVelocity(double velocityRPM) {
  }

  default void setIndexVoltage(double volts) {
  }

  default void stop() {
  }

  class DyeRotorInputs {
    public double spinPositionRotations;
    public double spinVelocityRPM;
    public double spinAppliedVolts;
    public double spinStatorCurrentAmps;
    public double spinSupplyCurrentAmps;
    public boolean spinMotorConnected = true;

    public double indexPositionRotations;
    public double indexVelocityRPM;
    public double indexAppliedVolts;
    public double indexStatorCurrentAmps;
    public double indexSupplyCurrentAmps;
    public boolean indexLeadMotorConnected = true;
    public boolean indexFollowerMotorConnected = true;
  }
}