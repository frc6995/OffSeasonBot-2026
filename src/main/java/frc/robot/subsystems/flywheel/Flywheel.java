package frc.robot.subsystems.flywheel;

import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotVisualizer;
import frc.robot.util.ShotController;
import frc.robot.util.ShotController.ShooterTargetData;

// import frc.robot.util.CtreUtil;

public class Flywheel extends SubsystemBase {
  public static class FlywheelConstants {
    // PID Constants
    public static final double kP = 0.1;
    // Feedforward Constants
    public static final double kS = 0.25;
    public static final double kV = 0.08;
    // CAN IDs
    public static final int kLeadMotorCANID = 40;
    public static final int kFollowMotor1CANID = 41;
    public static final int kFollowMotor2CANID = 42;
    public static final int kFollowMotor3CANID = 43;
    // Motor Config Constants
    public static final double kSupplyCurrentLimit = 40;
    public static final double kStatorCurrentLimit = 80;
    public static final double kNewMaxVoltage = 10;
    public static final double kNewMinVoltage = 0;
    public static final double kReduction = 1;
    public static final double kToleranceRPM = 100;
    public static final double FlywheelMOI = 0.000292639653; // meters^2 kg

    public static final double SAFE_SHOT = 0;
    // Sim Constants
    // public static final double kDiameter = 2;
    // public static final double kMass = 4.15;
    public static final double [][] kShooterData = {
      {0.0, 1500},
      {3.0, 1850},
      {4.0, 1950},
      {5.0, 2050},
      {10, 2500},
      {15.0, 3500}
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
  private final Supplier<ShooterTargetData> targetData;

  private FlywheelState flywheelState = FlywheelState.DISABLED;

  public Flywheel(FlywheelIO io, Supplier<ShooterTargetData> shotData) {
    this.io = io;
    this.targetData = shotData;
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

  @Logged(name = "Connected", importance = Importance.CRITICAL)
  public boolean areMotorsConnected() {
    return inputs.leadMotorConnected
        && inputs.followerMotor1Connected
        && inputs.followerMotor2Connected
        && inputs.followerMotor3Connected;
  }

  @Logged(name = "Velocity", importance = Importance.INFO)
  public double getVelocityRPM() {
    return inputs.velocityRPM;
  }

  @Logged(name = "Setpoint", importance = Importance.INFO)
  public double getSetpointRPM() {
    return targetData.get().flywheelRpm();
  }

  @Logged(name = "Voltage", importance = Importance.DEBUG)
  public double getAppliedVolts() {
    return inputs.appliedVolts;
  }

  @Override
  public void periodic() {

    if (DriverStation.isDisabled()) {
      // An ACTIVE request must never survive a disable. The request itself is legitimate, but the
      // thing that would normally clear it can't run: requestRobotIdle() is bound to the shoot
      // button's onFalse edge, and CommandScheduler refuses to schedule a non-runsWhenDisabled
      // command while the robot is disabled - so releasing the button after a disable does
      // nothing. Auto has the same hole: the scheduler cancels the running auto command at the end
      // of the period, so a later "stop shooting" event trigger never fires. Either way the wheel
      // would spin up the instant the robot is re-enabled, with nobody touching the controller.
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
      case ACTIVE -> targetData.get().flywheelRpm();
      case SAFE_SHOT -> FlywheelConstants.SAFE_SHOT;
    };
  }
}
