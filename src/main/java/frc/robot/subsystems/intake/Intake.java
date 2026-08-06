package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotVisualizer;

public class Intake extends SubsystemBase {

    public static final class IntakeConstants {
        public static final int kKICKER_MOTOR_ID = 34;
        public static final int kROLLER_LEAD_MOTOR_ID = 30;
        public static final int kROLLER_FOLLOWER_MOTOR_ID = 31;
        public static final int kEXTENSION_LEAD_MOTOR_ID = 32;
        public static final int kEXTENSION_FOLLOWER_MOTOR_ID = 33;

        // Kicker PID Constants
        public static final double kKickerP = 0.1;
        // Kicker Feedforward Constants
        public static final double kKickerS = 0.25;
        public static final double kKickerV = 0.08;
        // Kicker Config Constants
        public static final double kKickerSupplyCurrentLimit = 40;
        public static final double kKickerStatorCurrentLimit = 80;
        public static final double kKickerMaxVoltage = 10;
        public static final double kKickerMinVoltage = 0;
        public static final double kKickerReduction = 1.5;
        public static final double kKickerToleranceRPM = 10;
        public static final double kKickerMOI = 0.000292639653; // meters^2 kg

        // Roller PID Constants
        public static final double kRollerP = 0.1;
        // Roller Feedforward Constants
        public static final double kRollerS = 0.25;
        public static final double kRollerV = 0.08;
        // Roller Config Constants
        public static final double kRollerSupplyCurrentLimit = 40;
        public static final double kRollerStatorCurrentLimit = 80;
        public static final double kRollerMaxVoltage = 10;
        public static final double kRollerMinVoltage = 0;
        public static final double kRollerReduction = 3.45;
        public static final double kRollerToleranceRPM = 10;
        public static final double kRollerMOI = 0.000292639653; // meters^2 kg


        public static final double kExtensionStatorCurrentLimit = 80.0;
        public static final double kExtensionSupplyCurrentLimit = 40.0;
        public static final double kExtensionReduction = 3.33;

        public static final double kVelMetersPerSecond = 0.5;
        public static final double kAccelMetersPerSecondSquared = 0.5;

        public static final double kExtensionP = 20;
        public static final double kExtensionV = 0.07;

        public static final double kExtensionMaxMeters = 0.5;
        public static final double kExtensionMinMeters = 0.0;

        public static final double kDrumCircumferenceMeters = 0.119;

        public static final double kRollerForwardVolts = 4.0;
        public static final double kKickerForwardVolts = 4.0;

        public static final double kRollerEjectingRPM = -4.0;
        public static final double kKickerEjectingRPM = -4.0;

        public static final double kRollerForwardRPM = kRollerForwardVolts;
        public static final double kKickerForwardRPM = kKickerForwardVolts;
        public static final double acceleration = 5.0;
        public static final double velocity = 5.0;

        // Extension sweeps between these two positions while agitating,
        // swapping targets every kAgitateIntervalSeconds.
        public static final double kAgitateNearMeters = 0.2;
        public static final double kAgitateFarMeters = kExtensionMaxMeters;
        public static final double kAgitateIntervalSeconds = 0.35;
    }

    public enum IntakeState {
        RETRACTED,
        INTAKING,
        IDLE,
        AGITATING,
        EJECTING
    }

    private final IntakeIO io;
    private final IntakeIO.IntakeInputs inputs = new IntakeIO.IntakeInputs();

    private final MechanismLigament2d intakeLigament = new MechanismLigament2d("intake", Units.inchesToMeters(8), 10.854, 6,
            new Color8Bit(52, 235, 137));
            
    private IntakeState intakeState = IntakeState.IDLE;

    private final Timer agitateTimer = new Timer();
    private boolean agitateAtFarPosition = false;
    private double agitateNearMeters = IntakeConstants.kAgitateNearMeters;
    private double agitateFarMeters = IntakeConstants.kAgitateFarMeters;
    private double agitateIntervalSeconds = IntakeConstants.kAgitateIntervalSeconds;

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
        if (state == IntakeState.AGITATING && intakeState != IntakeState.AGITATING) {
            agitateAtFarPosition = false;
            agitateTimer.restart();
        }
        intakeState = state;
    }

    public IntakeState getState() {
        return intakeState;
    }

    public void retract() {
        setState(IntakeState.RETRACTED);
    }

    public void intakeFuel() {
        setState(IntakeState.INTAKING);
    }

    public void setIdle() {
        setState(IntakeState.IDLE);
    }

    public void agitate() {
        agitate(IntakeConstants.kAgitateNearMeters,
                IntakeConstants.kAgitateFarMeters,
                IntakeConstants.kAgitateIntervalSeconds);
    }

    /**
     * Sweeps the extension back and forth between two positions while the rollers
     * and kicker keep spinning forward.
     *
     * @param nearMeters      the retracted end of the sweep
     * @param farMeters       the extended end of the sweep
     * @param intervalSeconds how long to hold each end before swapping targets
     */
    public void agitate(double nearMeters, double farMeters, double intervalSeconds) {
        agitateNearMeters = clampExtension(nearMeters);
        agitateFarMeters = clampExtension(farMeters);
        agitateIntervalSeconds = Math.max(intervalSeconds, 0.02);
        setState(IntakeState.AGITATING);
    }

    public void eject() {
        setState(IntakeState.EJECTING);
    }

    public void resetEncoder() {
        io.resetEncoder();
    }

    public double getRollerVelocityRPM() {
        return inputs.rollerVelocityRPM;
    }

    public double getRollerAppliedVolts() {
        return inputs.rollerAppliedVolts;
    }

    public double getRollerStatorCurrentAmps() {
        return inputs.rollerStatorCurrentAmps;
    }

    public double getRollerSupplyCurrentAmps() {
        return inputs.rollerSupplyCurrentAmps;
    }

    public double getKickVelocityRPM() {
        return inputs.kickerVelocityRPM;
    }

    public double getKickAppliedVolts() {
        return inputs.kickerAppliedVolts;
    }

    public double getKickStatorCurrentAmps() {
        return inputs.kickerStatorCurrentAmps;
    }

    public double getKickSupplyCurrentAmps() {
        return inputs.kickerSupplyCurrentAmps;
    }

    public double getExtensionAppliedVolts() {
        return inputs.extensionAppliedVolts;
    }

    public double getExtensionStatorCurrentAmps() {
        return inputs.extensionStatorCurrentAmps;
    }

    public double getExtensionSupplyCurrentAmps() {
        return inputs.extensionSupplyCurrentAmps;
    }

    public boolean areRollerMotorsConnected() {
        return inputs.rollerLeadMotorConnected
                && inputs.rollerFollowerMotorConnected;
    }

    public boolean areExtensionMotorsConnected() {
        return inputs.extensionLeadMotorConnected
                && inputs.extensionFollowerMotorConnected;
    }

    public boolean isKickMotorConnected() {
        return inputs.kickerMotorConnected;
    }

    @Override
    public void periodic() {
    io.updateInputs(inputs);

    double retractedLengthMeters = Units.inchesToMeters(8.0);
    double extensionMeters = inputs.extensionPositionMeters;

    intakeLigament.setLength(retractedLengthMeters + extensionMeters);

        io.setKickerVelocity(resolveKickerTargetVelocity(intakeState));
        io.setRollerVelocity(resolveRollerTargetVelocity(intakeState));
        io.setExtensionPosition(resolveExtensionTargetPosition(intakeState));
    }

    private double resolveExtensionTargetPosition(IntakeState state) {
        return switch (state) {
            case IDLE -> IntakeConstants.kExtensionMinMeters;
            case RETRACTED -> IntakeConstants.kExtensionMinMeters;
            case INTAKING -> IntakeConstants.kExtensionMaxMeters;
            case AGITATING -> resolveAgitationTargetPosition();
            case EJECTING -> IntakeConstants.kExtensionMaxMeters;
        };
    }

    private double resolveAgitationTargetPosition() {
        if (agitateTimer.advanceIfElapsed(agitateIntervalSeconds)) {
            agitateAtFarPosition = !agitateAtFarPosition;
        }
        return agitateAtFarPosition ? agitateFarMeters : agitateNearMeters;
    }

    private static double clampExtension(double positionMeters) {
        return MathUtil.clamp(
                positionMeters,
                IntakeConstants.kExtensionMinMeters,
                IntakeConstants.kExtensionMaxMeters);
    }

    private static double resolveRollerTargetVelocity(IntakeState state) {
        return switch (state) {
            case IDLE -> 0.0;
            case RETRACTED -> 0.0;
            case INTAKING -> IntakeConstants.kRollerForwardRPM;
            case AGITATING -> IntakeConstants.kRollerForwardRPM;
            case EJECTING -> IntakeConstants.kRollerEjectingRPM;

        };
    }

    private static double resolveKickerTargetVelocity(IntakeState state) {
        return switch (state) {
            case IDLE -> 0.0;
            case RETRACTED -> 0.0;
            case INTAKING -> IntakeConstants.kKickerForwardRPM;
            case AGITATING -> IntakeConstants.kKickerForwardRPM;
            case EJECTING -> IntakeConstants.kKickerEjectingRPM;

        };
    }
}