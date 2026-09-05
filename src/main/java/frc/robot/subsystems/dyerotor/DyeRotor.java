package frc.robot.subsystems.dyerotor;

import edu.wpi.first.epilogue.Logged;
import frc.robot.util.ArrayUtil;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotVisualizer;

public class DyeRotor extends SubsystemBase {
    public static final class DyeRotorConstants {

        public static final int kSpinMotorCANID = 23; // Tune
        public static final int kLeadIndexMotorCANID = 21; // Tune
        public static final int kFollowIndexMotorCANID = 22; // Tune

        public static final double kSpinReduction = 35;
        public static final double kIndexReduction = 2.5;
        public static final double kSpinMOI = 0.004;
        public static final double kIndexMOI = 0.002;

        public static final double kSpinStatorCurrentLimit = 60.0;
        public static final double kSpinSupplyCurrentLimit = 40.0;
        public static final double kIndexStatorCurrentLimit = 60.0;
        public static final double kIndexSupplyCurrentLimit = 40.0;

        public static final double kSpinKP = 5.0;
        public static final double kSpinKS = 0.31;
        public static final double kSpinKV = 4.0;

        public static final double kIndexKP = 0.5;
        public static final double kIndexKS = 0.3;
        public static final double kIndexKV = 0.31;

        public static final double kSpinForwardRPM = 120.0;
        public static final double kSpinBackwardRPM = 30.0;
        public static final double kSpinVelocityToleranceRPM = 20.0;

        public static final double kIndexForwardRPM = 1000.0;
        public static final double kIndexBackwardRPM = 30.0;

        // Delay after the state is set to shoot before each mechanism spins up.
        public static final double kIndexSpinUpDelaySecs = 0.001;
        public static final double kSpinSpinUpDelaySecs = 0.001;

        public static final double kMinAppliedVolts = 0.0;
        public static final double kMaxAppliedVolts = 10.0;

        public static final double kSpinIdleReverseVolts = -10.0;

        private DyeRotorConstants() {
        }
    }

    public enum DyeRotorState {
        IDLE,
        SPIN;
    }

    private final DyeRotorIO io;
    private final DyeRotorIO.DyeRotorInputs inputs = new DyeRotorIO.DyeRotorInputs();

    private DyeRotorState spinState = DyeRotorState.IDLE;
    private DyeRotorState indexState = DyeRotorState.IDLE;

    private static final double kLoopPeriodSecs = 0.02;

    // Ticks remaining before the index (rollers) / spin motor are allowed to spin up.
    private int indexSpinUpTicksRemaining = 0;
    private int spinSpinUpTicksRemaining = 0;

    public DyeRotor() {
        this(new DyeRotorIO() {
        });
    }

    public DyeRotor(DyeRotorIO io) {
        this.io = io;
    }

    public void stop() {
        cancelPendingSpinUp();
        spinState = DyeRotorState.IDLE;
        indexState = DyeRotorState.IDLE;
    }

    public void setState(DyeRotorState state) {
        spinState = state;
        indexState = state;
    }

    public void requestIdle() {
        cancelPendingSpinUp();
        setState(DyeRotorState.IDLE);
    }

    /** Requests SPIN, delaying the rollers (index) and the hood motor after the request depending on their variables. */
    public void requestSpin() {
        indexState = DyeRotorState.IDLE;
        spinState = DyeRotorState.IDLE;
        indexSpinUpTicksRemaining = ticksFor(DyeRotorConstants.kIndexSpinUpDelaySecs);
        spinSpinUpTicksRemaining = ticksFor(DyeRotorConstants.kSpinSpinUpDelaySecs);
    }

    private void cancelPendingSpinUp() {
        indexSpinUpTicksRemaining = 0;
        spinSpinUpTicksRemaining = 0;
    }

    private static int ticksFor(double seconds) {
        return (int) Math.ceil(seconds / kLoopPeriodSecs);
    }

    @Logged(name = "Spin State", importance = Importance.CRITICAL)
    public DyeRotorState getSpinState() {
        return spinState;
    }

    @Logged(name = "Index State", importance = Importance.CRITICAL)
    public DyeRotorState getIndexState() {
        return indexState;
    }

    @Logged(name = "Connected", importance = Importance.CRITICAL)
    public boolean isConnected() {
        return inputs.indexLeadMotorConnected && inputs.indexFollowerMotorConnected && inputs.spinMotorConnected;
    }

    @Logged(name = "Spin Velocity", importance = Importance.INFO)
    public double getSpinVelocityRPM() {
        return inputs.spinVelocityRPM;
    }

    @Logged(name = "Index Velocity", importance = Importance.INFO)
    public double getIndexVelocityRPM() {
        return inputs.indexVelocityRPM;
    }


    /*
     * Supply current totals. Supply, not stator: stator current is measured on the motor side of
     * the controller and can be several times what is drawn from the battery, so a stator sum
     * overstates the power budget. These are the series tools/power_analysis charts.
     */

    @Logged(name = "Index/Supply Current Total", importance = Importance.CRITICAL)
    public double getIndexTotalSupplyCurrentAmps() {
        return ArrayUtil.sum(inputs.indexMotorSupplyCurrentAmps);
    }

    /** Single motor, so this is just the spin motor's draw; named for consistency. */
    @Logged(name = "Spin/Supply Current Total", importance = Importance.CRITICAL)
    public double getSpinTotalSupplyCurrentAmps() {
        return inputs.spinSupplyCurrentAmps;
    }

    /** Spin and indexer combined - the dye rotor's line in the robot's power budget. */
    @Logged(name = "Supply Current Total", importance = Importance.CRITICAL)
    public double getTotalSupplyCurrentAmps() {
        return getSpinTotalSupplyCurrentAmps() + getIndexTotalSupplyCurrentAmps();
    }

    /** Per-motor detail for the indexer pair, indexed [lead, follower]. */
    @Logged(name = "Index/Supply Currents", importance = Importance.DEBUG)
    public double[] getIndexMotorSupplyCurrentsAmps() {
        return inputs.indexMotorSupplyCurrentAmps;
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            // A SPIN request must never survive a disable
            setState(DyeRotorState.IDLE);
        }

        if (indexSpinUpTicksRemaining > 0 && --indexSpinUpTicksRemaining <= 0) {
            indexState = DyeRotorState.SPIN;
        }
        if (spinSpinUpTicksRemaining > 0 && --spinSpinUpTicksRemaining <= 0) {
            spinState = DyeRotorState.SPIN;
        }

        io.updateInputs(inputs);

        io.setSpinVelocity(resolveSpinTargetRPM(spinState));
        io.setIndexVelocity(resolveIndexTargetRPM(indexState));
    }

  @Override
  public void simulationPeriodic() {
    RobotVisualizer.updateHook(inputs.spinVelocityRPM * 2 * Math.PI / 60.0 * 0.02);
  }

    private static double resolveSpinTargetRPM(DyeRotorState state) {
        return switch (state) {
            case IDLE -> -DyeRotorConstants.kSpinBackwardRPM;
            case SPIN -> DyeRotorConstants.kSpinForwardRPM;
        };
    }

    private static double resolveIndexTargetRPM(DyeRotorState state) {
        return switch (state) {
            case IDLE -> 0.0;
            case SPIN -> DyeRotorConstants.kIndexForwardRPM;
        };
    }
}