package frc.robot.subsystems.dyerotor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DyeRotor extends SubsystemBase {
  public enum State {
    IDLE,
    SPIN,
    SPIN_BACKWARDS
  }

  public static final class DyeRotorConstants {
    public static final double kSpinForwardRPM = 0.0;
    public static final double kSpinBackwardRPM = 0.0;
    public static final double kSpinVelocityToleranceRPM = 0.0;

    public static final double kIndexForwardVolts = 0.0;
    public static final double kIndexBackwardVolts = 0.0;

    private DyeRotorConstants() {
    }
  }

  private final DyeRotorIO io;
  private final DyeRotorIO.DyeRotorInputs inputs = new DyeRotorIO.DyeRotorInputs();

  private State spinState = State.IDLE;
  private State indexState = State.IDLE;

  public DyeRotor() {
    this(new DyeRotorIO() {
    });
  }

  public DyeRotor(DyeRotorIO io) {
    this.io = io;
  }

  public void stop() {
    spinState = State.IDLE;
    indexState = State.IDLE;
  }

  public void setState(State state) {
    spinState = state;
    indexState = state;
  }

  public void setSpinState(State state) {
    spinState = state;
  }

  public void setIndexState(State state) {
    indexState = state;
  }

  public State getSpinState() {
    return spinState;
  }

  public State getIndexState() {
    return indexState;
  }

  public double getSpinVelocityRPM() {
    return inputs.spinVelocityRPM;
  }

  public double getIndexVoltage() {
    return inputs.indexAppliedVolts;
  }

  public boolean isSpinReady() {
    return MathUtil.isNear(
        resolveSpinTargetRPM(spinState),
        inputs.spinVelocityRPM,
        DyeRotorConstants.kSpinVelocityToleranceRPM);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    io.setSpinVelocity(resolveSpinTargetRPM(spinState));
    io.setIndexVoltage(resolveIndexTargetVoltage(indexState));
  }

  private static double resolveSpinTargetRPM(State state) {
    return switch (state) {
      case IDLE -> 0.0;
      case SPIN -> DyeRotorConstants.kSpinForwardRPM;
      case SPIN_BACKWARDS -> -DyeRotorConstants.kSpinBackwardRPM;
    };
  }

  private static double resolveIndexTargetVoltage(State state) {
    return switch (state) {
      case IDLE -> 0.0;
      case SPIN -> DyeRotorConstants.kIndexForwardVolts;
      case SPIN_BACKWARDS -> -DyeRotorConstants.kIndexBackwardVolts;
    };
  }
}