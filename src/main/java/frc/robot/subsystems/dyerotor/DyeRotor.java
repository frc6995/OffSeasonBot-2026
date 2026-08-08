package frc.robot.subsystems.dyerotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotVisualizer;

public class DyeRotor extends SubsystemBase {
  public static final class DyeRotorConstants {

    public static final int kSpinMotorCANID = 23; // Tune
    public static final int kLeadIndexMotorCANID = 21; // Tune
    public static final int kFollowIndexMotorCANID = 22; // Tune

    public static final double kSpinReduction = 36.0;
    public static final double kIndexReduction = 2.5;
    public static final double kSpinMOI = 0.004;
    public static final double kIndexMOI = 0.002;

    public static final double kSpinStatorCurrentLimit = 60.0;
    public static final double kSpinSupplyCurrentLimit = 40.0;
    public static final double kIndexStatorCurrentLimit = 60.0;
    public static final double kIndexSupplyCurrentLimit = 40.0;

    public static final double kSpinKP = 0.25;
    public static final double kSpinKS = 0.0;
    public static final double kSpinKV = 0.30;

    public static final double kIndexKP = 0.25;
    public static final double kIndexKS = 0.0;
    public static final double kIndexKV = 0.30;

    public static final double kSpinForwardRPM = 120.0;
    public static final double kSpinBackwardRPM = 30.0;
    public static final double kSpinVelocityToleranceRPM = 20.0;

    public static final double kIndexForwardRPM = 300.0;
    public static final double kIndexBackwardRPM = 300.0;

    private DyeRotorConstants() {
    }
  }

  public enum DyeRotorState {
    IDLE,
    SPIN,
    SPIN_BACKWARDS
  }

  private final DyeRotorIO io;
  private final DyeRotorIO.DyeRotorInputs inputs = new DyeRotorIO.DyeRotorInputs();

  private DyeRotorState spinState = DyeRotorState.IDLE;
  private DyeRotorState indexState = DyeRotorState.IDLE;

  public DyeRotor() {
    this(new DyeRotorIO() {
    });
  }

  public DyeRotor(DyeRotorIO io) {
    this.io = io;
  }

  public void stop() {
    spinState = DyeRotorState.IDLE;
    indexState = DyeRotorState.IDLE;
  }

  public void setState(DyeRotorState state) {
    spinState = state;
    indexState = state;
  }

  public void setSpinState(DyeRotorState state) {
    spinState = state;
  }

  public void setIndexState(DyeRotorState state) {
    indexState = state;
  }

  public DyeRotorState getSpinState() {
    return spinState;
  }

  public DyeRotorState getIndexState() {
    return indexState;
  }

  public double getSpinVelocityRPM() {
    return inputs.spinVelocityRPM;
  }

  public double getIndexVoltage() {
    return inputs.indexAppliedVolts;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    io.setSpinVelocity(resolveSpinTargetRPM(spinState));
    io.setIndexVelocity(resolveIndexTargetRPM(indexState));

    RobotVisualizer.updateHook(inputs.spinVelocityRPM * 2 * Math.PI / 60.0 * 0.02);
  }

  @Override
  public void simulationPeriodic() {
    RobotVisualizer.updateHook(inputs.spinVelocityRPM * 2 * Math.PI / 60.0 * 0.02);
  }

  private static double resolveSpinTargetRPM(DyeRotorState state) {
    return switch (state) {
      case IDLE -> 0.0;
      case SPIN -> DyeRotorConstants.kSpinForwardRPM;
      case SPIN_BACKWARDS -> -DyeRotorConstants.kSpinBackwardRPM;
    };
  }

  private static double resolveIndexTargetRPM(DyeRotorState state) {
    return switch (state) {
      case IDLE -> 0.0;
      case SPIN -> DyeRotorConstants.kIndexForwardRPM;
      case SPIN_BACKWARDS -> 0.0;
    };
  }
}