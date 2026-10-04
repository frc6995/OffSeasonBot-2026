package frc.robot.subsystems.flywheel;

import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
  public static class FlywheelConstants {
    // PID Constants
    public static final double kP = 0.25;
    // Feedforward Constants
    public static final double kS = 0.45;
    public static final double kV = 0.1;
    // CAN IDs
    public static final int kLeadMotorCANID = 40;
    public static final int kFollowMotor1CANID = 41;
    public static final int kFollowMotor2CANID = 42;
    public static final int kFollowMotor3CANID = 43;
    // Motor Config Constants
    public static final double kSupplyCurrentLimitAmps = 120;
    public static final double kStatorCurrentLimitAmps = 120;
    public static final double kNewMaxVolts = 10;
    public static final double kNewMinVolts = 0;
    public static final double kReduction = 1;
    public static final double kToleranceRPM = 100;
    public static final double FlywheelMOI = 0.000292639653; // meters^2 kg

    public static final double kSafeShotRPM = 2700;

    public static final double [][] kShooterData = {
      {0.0, 2100},
      {1.0, 2100},
      {1.5, 2100},
      {2.0, 2180},
      {2.5, 2300},
      {3.0, 2400},
      {3.5, 2500},
      {4.0, 2580},
      {4.5, 2660},
      {5.0, 2780},
      {6.0, 3000},
      {7.0, 3200}
    };

    // distance from POI.PASSING_WALL
    public static final double [][] kPassingShooterData = {
      {0.0, 1500},
      {3.0, 1850},
      {4.0, 1950},
      {5.0, 2050},
      {10, 2500},
      {15.0, 3500}
    };

  }

  public enum FlywheelState {
    DISABLED,
    ACTIVE,
    SAFE_SHOT
  }

  private final FlywheelIO io;
  private final FlywheelIO.FlywheelInputs inputs = new FlywheelIO.FlywheelInputs();
  private final Supplier<Double> targetRpm;

  private FlywheelState flywheelState = FlywheelState.DISABLED;

  public Flywheel(FlywheelIO io, Supplier<Double> targetRpm) {
    this.io = io;
    this.targetRpm = targetRpm;
  }

  public void setState(FlywheelState state) {
    flywheelState = state;
  }

  public void requestDisable() {
    setState(FlywheelState.DISABLED);
  }

  public void requestActive() {
    setState(FlywheelState.ACTIVE);
  }

  public void stop() {
    flywheelState = FlywheelState.DISABLED;

  }

  @Logged(name = "State", importance = Importance.CRITICAL)
  public FlywheelState getState() {
    return flywheelState;
  }

  @Logged(name = "Velocity", importance = Importance.INFO)
  public double getVelocityRPM() {
    return inputs.velocityRPM;
  }

  @Logged(name = "Setpoint", importance = Importance.INFO)
  public double getSetpointRPM() {
    return resolveTargetRPM(flywheelState);
  }

  @Logged(name = "Voltage", importance = Importance.DEBUG)
  public double getAppliedVolts() {
    return inputs.appliedVolts;
  }

  @Logged(name = "Stator Currents", importance = Importance.DEBUG)
  public double[] getMotorStatorCurrentsAmps() {
    return inputs.motorStatorCurrentAmps;
  }

  @Override
  public void periodic() {

    if (DriverStation.isDisabled()) {
      setState(FlywheelState.DISABLED);
    }

    io.updateInputs(inputs);

    // DISABLED coasts the flywheel down instead of holding 0 RPM with closed-loop control.
    if (flywheelState == FlywheelState.DISABLED) {
      io.stop();
    } else {
      io.setVelocityRPM(resolveTargetRPM(flywheelState));
    }
  }

  // shoot is NOT 10000 rpm
  private double resolveTargetRPM(FlywheelState state) {
    return switch (state) {
      case DISABLED -> 0.0;
      case ACTIVE -> targetRpm.get();
      case SAFE_SHOT -> FlywheelConstants.kSafeShotRPM;
    };
  }
}
