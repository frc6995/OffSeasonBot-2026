package frc.robot.subsystems.intake;

import edu.wpi.first.epilogue.Logged;
import frc.robot.util.ArrayUtil;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotVisualizer;
import frc.robot.util.currentlimit.CurrentLimit;

public class Intake extends SubsystemBase {

    public static final class IntakeConstants {
        public static final int kKICKER_MOTOR_ID = 34;
        public static final int kROLLER_LEAD_MOTOR_ID = 30;
        public static final int kROLLER_FOLLOWER_MOTOR_ID = 31;
        public static final int kEXTENSION_LEAD_MOTOR_ID = 32;
        public static final int kEXTENSION_FOLLOWER_MOTOR_ID = 33;

        // // Kicker PID Constants
        // public static final double kKickerP = 0.2;
        // // Kicker Feedforward Constants
        // public static final double kKickerS = 0.25;
        // public static final double kKickerV = 0.164;
        // Kicker Config Constants
        public static final double kKickerSupplyCurrentLimitAmps = 40;
        public static final double kKickerStatorCurrentLimitAmps = 80;
        public static final double kKickerMaxVolts = 10;
        public static final double kKickerMinVolts = -10;
        public static final double kKickerReduction = 1.5;
        public static final double kKickerToleranceRPM = 10;
        public static final double kKickerMOI = 0.0000292639653; // meters^2 kg
        public static final double kKickerEjectingVoltage = -10.0;
        public static final double kKickerForwardVoltage = 10.0;
        public static final double kKickerIdleVolts = 0.0;

        // // Roller PID Constants
        // public static final double kRollerP = 0.2;
        // // Roller Feedforward Constants
        // public static final double kRollerS = 0.25;
        // public static final double kRollerV = 0.396;
        // Roller Config Constants
        public static final double kRollerSupplyCurrentLimitAmps = 40;
        public static final double kRollerStatorCurrentLimitAmps = 80;
        public static final double kRollerMaxVolts = 10;
        public static final double kRollerMinVolts = -10;
        public static final double kRollerReduction = 3.45;
        public static final double kRollerToleranceRPM = 10;
        public static final double kRollerMOI = 0.0000292639653; // meters^2 kg
        public static final double kRollerEjectingVoltage = -10.0;
        public static final double kRollerForwardVoltage = 10.0;
        public static final double kRollerIdleVolts = 2.0;

        // Extension PID Constants
        public static final double kExtensionP = 5.0;
        // Extension Feedforward Constants
        public static final double kExtensionV = 0.07;
        // Extension Config Constants
        public static final double kExtensionStatorCurrentLimitAmps = 20.0;
        public static final double kExtensionSupplyCurrentLimitAmps = 20.0;
        public static final double kExtensionReduction = 3.33;
        public static final double kExtensionMaxMeters = IntakeIOTalonFX.mechanismRotationsToMeters(3.83);
        public static final double kExtensionMinMeters = 0.0;
        public static final double kIntakeAngleDegrees = 10.8;
        public static final double kDrumCircumferenceMeters = 0.119;
        public static final double kExtensionAccelerationRotationsPerSec2 = 200.0;
        public static final double kExtensionCruiseVelocityRotationsPerSec = 10.0;

        // Full agitate sweeps down from 100% to 50% extension and holds once it arrives
        // (shoot-only). Mini agitate oscillates between 70% and 100% every
        // kMiniAgitateIntervalSeconds (both shoot+intake pressed while scoring).
        public static final double kFullAgitateNearMeters = IntakeIOTalonFX.mechanismRotationsToMeters(2.45);
        public static final double kFullAgitateFarMeters = kExtensionMaxMeters;
        public static final double kMiniAgitateNearMeters = IntakeIOTalonFX.mechanismRotationsToMeters(3.4);
        public static final double kMiniAgitateFarMeters = kExtensionMaxMeters;
        public static final double kMiniAgitateIntervalSeconds = 0.3;
        public static final double kFullAgitateIntervalSeconds = 0.4;
        public static final double kAgitateToleranceMeters = 0.02;
    }

    public enum IntakeState {
        RETRACTED,
        ACTIVE,
        IDLE,
        MINI_AGITATE,
        FULL_AGITATE,
        EJECTING
    }

    private final IntakeIO io;
    private final IntakeIO.IntakeInputs inputs = new IntakeIO.IntakeInputs();

    private boolean isZeroed = false;

    private final MechanismLigament2d intakeLigament = new MechanismLigament2d("intake", Units.inchesToMeters(8), 10.854, 6,
            new Color8Bit(52, 235, 137));
            
    private IntakeState intakeState = IntakeState.RETRACTED;

    // sim visualization
    private double commandedExtensionMeters;

    private final Timer agitateTimer = new Timer();
    private boolean agitateAtNearPosition = false;

    public Intake() {
        this(new IntakeIO() {
        });
    }

    public Intake(IntakeIO io) {
        this.io = io;
        RobotVisualizer.addIntake(intakeLigament);
    }

    public void stop() {
        intakeState = IntakeState.IDLE;
        io.stop();
    }

    public void setState(IntakeState state) {
        if ((state == IntakeState.MINI_AGITATE || state == IntakeState.FULL_AGITATE) && intakeState != IntakeState.MINI_AGITATE && intakeState != IntakeState.FULL_AGITATE) {
            agitateAtNearPosition = false;
            agitateTimer.restart();
        }
        intakeState = state;
    }

    public void requestRetract() {
        setState(IntakeState.RETRACTED);
    }

    public void requestActive() {
        setState(IntakeState.ACTIVE);
    }

    public void requestIdle() {
        setState(IntakeState.IDLE);
    }

    public void requestMiniAgitate() {
        setState(IntakeState.MINI_AGITATE);
    }

    public void requestFullAgitate() {
        setState(IntakeState.FULL_AGITATE);
    }

    public void requestEject() {
        setState(IntakeState.EJECTING);
    }

    public void resetEncoder() {
        io.resetEncoder();
        isZeroed = true;
    }

    public boolean isZeroed(){
        return isZeroed;
    }

    public void setRollerCurrentLimit(CurrentLimit limit) {
        io.setRollerCurrentLimits(limit.statorCurrentLimitAmps(), limit.supplyCurrentLimitAmps());
    }

    public void setKickerCurrentLimit(CurrentLimit limit) {
        io.setKickerCurrentLimits(limit.statorCurrentLimitAmps(), limit.supplyCurrentLimitAmps());
    }

    @Logged(name = "State", importance = Importance.CRITICAL)
    public IntakeState getState() {
        return intakeState;
    }

    @Logged(name = "Extension/Position", importance =  Importance.INFO)
    public double getExtensionPositionMeters() {
        return inputs.extensionPositionMeters;
    }

    @Logged(name = "Roller/Voltage", importance = Importance.DEBUG)
    public double getRollerAppliedVolts() {
        return inputs.rollerAppliedVolts;
    }

    @Logged(name = "Kicker/Voltage", importance =  Importance.DEBUG)
    public double getKickAppliedVolts() {
        return inputs.kickerAppliedVolts;
    }

    @Logged(name = "Roller/Stator Current", importance =  Importance.DEBUG)
    public double getRollerStatorCurrentAmps() {
        return inputs.rollerStatorCurrentAmps;
    }

    @Logged(name = "Roller/Supply Current", importance =  Importance.DEBUG)
    public double getRollerSupplyCurrentAmps() {
        return inputs.rollerSupplyCurrentAmps;
    }

    @Logged(name = "Kicker/Stator Current", importance =  Importance.DEBUG)
    public double getKickStatorCurrentAmps() {
        return inputs.kickerStatorCurrentAmps;
    }

    @Logged(name = "Kicker/Supply Current", importance =  Importance.DEBUG)
    public double getKickSupplyCurrentAmps() {
        return inputs.kickerSupplyCurrentAmps;
    }

    @Logged(name = "Extension/Stator Current", importance =  Importance.DEBUG)
    public double getExtensionStatorCurrentAmps() {
        return inputs.extensionStatorCurrentAmps;
    }

    @Logged(name = "Extension/Supply Current", importance =  Importance.DEBUG)
    public double getExtensionSupplyCurrentAmps() {
        return inputs.extensionSupplyCurrentAmps;
    }

    /*
     * Per-mechanism supply current totals, including the follower motors that the single-motor
     * getters above miss. These are the series tools/power_analysis charts. Supply, not stator:
     * stator current is measured on the motor side of the controller and can be several times what
     * is actually drawn from the battery, so a stator sum overstates the power budget.
     */

    @Logged(name = "Roller/Supply Current Total", importance = Importance.CRITICAL)
    public double getRollerTotalSupplyCurrentAmps() {
        return ArrayUtil.sum(inputs.rollerMotorSupplyCurrentAmps);
    }

    @Logged(name = "Extension/Supply Current Total", importance = Importance.CRITICAL)
    public double getExtensionTotalSupplyCurrentAmps() {
        return ArrayUtil.sum(inputs.extensionMotorSupplyCurrentAmps);
    }

    /** Single motor, so this equals {@link #getKickSupplyCurrentAmps()}; named for consistency. */
    @Logged(name = "Kicker/Supply Current Total", importance = Importance.CRITICAL)
    public double getKickerTotalSupplyCurrentAmps() {
        return inputs.kickerSupplyCurrentAmps;
    }

    /** Roller, extension, and kicker combined - the intake's line in the robot's power budget. */
    @Logged(name = "Supply Current Total", importance = Importance.CRITICAL)
    public double getTotalSupplyCurrentAmps() {
        return getRollerTotalSupplyCurrentAmps()
                + getExtensionTotalSupplyCurrentAmps()
                + getKickerTotalSupplyCurrentAmps();
    }

    /** Per-motor detail, indexed [lead, follower]. */
    @Logged(name = "Roller/Supply Currents", importance = Importance.DEBUG)
    public double[] getRollerMotorSupplyCurrentsAmps() {
        return inputs.rollerMotorSupplyCurrentAmps;
    }

    @Logged(name = "Extension/Supply Currents", importance = Importance.DEBUG)
    public double[] getExtensionMotorSupplyCurrentsAmps() {
        return inputs.extensionMotorSupplyCurrentAmps;
    }

    public boolean isDeployed() {
        return getState() != IntakeState.RETRACTED;
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            //Reset to idle 
            setState(IntakeState.IDLE);
        }

        io.updateInputs(inputs);

        io.setKickerVoltage(resolveKickerTargetVoltage(intakeState));
        io.setRollerVoltage(resolveRollerTargetVoltage(intakeState));
        io.setExtensionPosition(clampExtension(resolveExtensionTargetPosition(intakeState)));
    }

    @Override
    public void simulationPeriodic() {
        double retractedLengthMeters = Units.inchesToMeters(8.0);
        double extensionMeters = inputs.extensionPositionMeters;
        intakeLigament.setLength(retractedLengthMeters + extensionMeters);
        RobotVisualizer.updateIntakeExtension(extensionMeters);
    }

    private double resolveExtensionTargetPosition(IntakeState state) {
        return switch (state) {
            case IDLE -> IntakeConstants.kExtensionMaxMeters;
            case RETRACTED -> IntakeConstants.kExtensionMinMeters;
            case ACTIVE -> IntakeConstants.kExtensionMaxMeters;
            case MINI_AGITATE -> resolveMiniAgitateTargetPosition();
            case FULL_AGITATE -> resolveFullAgitateTargetPosition();
            case EJECTING -> IntakeConstants.kExtensionMaxMeters;
        };
    }

    private double resolveMiniAgitateTargetPosition() {
        if (agitateTimer.advanceIfElapsed(IntakeConstants.kMiniAgitateIntervalSeconds)) {
            agitateAtNearPosition = !agitateAtNearPosition;
        }
        return agitateAtNearPosition ? IntakeConstants.kMiniAgitateNearMeters : IntakeConstants.kMiniAgitateFarMeters;
    }

    private double resolveFullAgitateTargetPosition() {
        if (!MathUtil.isNear(IntakeConstants.kFullAgitateNearMeters, inputs.extensionPositionMeters, IntakeConstants.kAgitateToleranceMeters) && agitateTimer.advanceIfElapsed(IntakeConstants.kFullAgitateIntervalSeconds)) {
            agitateAtNearPosition = !agitateAtNearPosition;
        }
        return agitateAtNearPosition ? IntakeConstants.kFullAgitateNearMeters : IntakeConstants.kFullAgitateFarMeters;
    }

    private static double clampExtension(double positionMeters) {
        return MathUtil.clamp(
                positionMeters,
                IntakeConstants.kExtensionMinMeters,
                IntakeConstants.kExtensionMaxMeters);
    }

    private double resolveRollerTargetVoltage(IntakeState state) {
        return switch (state) {
            case IDLE -> IntakeConstants.kRollerIdleVolts;
            case RETRACTED -> 0.0;
            case ACTIVE -> IntakeConstants.kRollerForwardVoltage;
            case MINI_AGITATE -> IntakeConstants.kRollerForwardVoltage;
            case FULL_AGITATE -> IntakeConstants.kRollerIdleVolts;
            case EJECTING -> IntakeConstants.kRollerEjectingVoltage;

        };
    }

    private double resolveKickerTargetVoltage(IntakeState state) {
        return switch (state) {
            case IDLE -> IntakeConstants.kKickerIdleVolts;
            case RETRACTED -> 0.0;
            case ACTIVE -> IntakeConstants.kKickerForwardVoltage;
            case MINI_AGITATE -> IntakeConstants.kKickerForwardVoltage;
            case FULL_AGITATE -> IntakeConstants.kKickerIdleVolts;
            case EJECTING -> IntakeConstants.kKickerEjectingVoltage;

        };
    }
}
