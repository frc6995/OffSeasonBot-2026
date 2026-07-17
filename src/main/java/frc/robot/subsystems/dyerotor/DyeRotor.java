package frc.robot.subsystems.dyerotor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DyeRotor extends SubsystemBase {
  public class DyeRotorConstants {
    public static final int kSpinMotorCANID = 23;
    public static final int kLeadIndexMotorCANID = 21;
    public static final int kFollowIndexMotorCANID = 22;

    public static final double kSpinReduction = 2.5;
    public static final double kIndexReduction = 36;

    public static final double kSpinSupplyCurrentLimit = 80; // Tune
    public static final double kSpinStatorCurrentLimit = 60; // Tune
    public static final double kIndexSupplyCurrentLimit = 80; // Tune
    public static final double kIndexStatorCurrentLimit = 60; // Tune

    public static final double kSpinKP = 4.0; // Tune
    public static final double kSpinKV = 0.0; // Tune
    public static final double kSpinKS = 0.0; // Tune

    public static final double SpinMOI = 0.091011;
    public static final double IndexMOI = 0.000534;

    public static final double kSpinForwardRPM = 3000.0; // Tune
    public static final double kSpinBackwardRPM = -1000.0; // Tune
    public static final double kSpinVelocityToleranceRPM = 100.0; // Tune
    
    public static final double kIndexForwardRPM = 100.0; // Tune
    public static final double kIndexBackwardRPM = -50.0; // Tune

    public static final double kIndexKP = 0.0; // Tune
    public static final double kIndexKV = 0.0; // Tune
    public static final double kIndexKS = 0.0; // Tune

    // public static final double kIndexForwardVolts = 8.0;
    // public static final double kIndexBackwardVolts = -4.0;
  }

  public enum State {
    IDLE,
    SPIN,
    SPIN_BACKWARDS
  }

  private final DyeRotorIO io;
  private final DyeRotorIO.DyeRotorInputs inputs = new DyeRotorIO.DyeRotorInputs();

  private State spinState = State.IDLE;
  private State indexState = State.IDLE;

  public DyeRotor() {
    this(new DyeRotorIO() {});
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
  public double getIndexVelocityRPM() {
    return inputs.indexVelocityRPM;
  }

  public double getSpinAppliedVolts() {
    return inputs.spinAppliedVolts;
  }
  
  public double getIndexVoltage() {
    return inputs.indexAppliedVolts;
  }

  public boolean isSpinMotorConnected() {
    return inputs.spinMotorConnected;
  }

  public boolean areIndexMotorsConnected() {
    return inputs.indexLeadMotorConnected
        && inputs.indexFollowerMotorConnected;
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
    // io.setIndexVoltage(resolveIndexTargetVoltage(indexState));
    io.setIndexVelocity(resolveIndexTargetRPM(indexState));
  }

  private static double resolveSpinTargetRPM(State state) {
    return switch (state) {
      case IDLE -> 0.0;
      case SPIN -> DyeRotorConstants.kSpinForwardRPM;
      case SPIN_BACKWARDS -> DyeRotorConstants.kSpinBackwardRPM;
      default -> 0.0;
    };
  }
  private static double resolveIndexTargetRPM(State state) {
    return switch (state) {
      case IDLE -> 0.0;
      case SPIN -> DyeRotorConstants.kIndexForwardRPM;
      case SPIN_BACKWARDS -> DyeRotorConstants.kIndexBackwardRPM;
      default -> 0.0;
    };
  }
  // private static double resolveIndexTargetVoltage(State state) {
  //   return switch (state) {
  //     case IDLE -> 0.0;
  //     case SPIN -> DyeRotorConstants.kIndexForwardVolts;
  //     case SPIN_BACKWARDS -> DyeRotorConstants.kIndexBackwardVolts;
  //     default -> 0.0;
  //   };
  // }
}