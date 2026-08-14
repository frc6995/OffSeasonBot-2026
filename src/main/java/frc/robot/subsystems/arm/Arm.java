package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotVisualizer;

public class Arm extends SubsystemBase {

    public static final class ArmConstants {
        // Elevator motor ids
        public static final int kELEVATOR_LEAD_MOTOR_ID = 57;
        public static final int kELEVATOR_FOLLOWER_MOTOR_ID = 60;

        // Elevator gains
        public static final double kElevatorP = 20.0;
        public static final double kElevatorS = 0.0;
        public static final double kElevatorV = 0.07;
        public static final double kElevatorG = 0.0;

        // Elevator configuration
        public static final double kElevatorStatorCurrentLimit = 80.0;
        public static final double kElevatorSupplyCurrentLimit = 40.0;
        public static final double kElevatorReduction = 20.0;
        public static final double kElevatorMinMeters = 0.0;
        public static final double kElevatorMaxMeters = 1.0;
        public static final double kElevatorCruiseVelocity = 5.0;
        public static final double kElevatorAcceleration = 20.0;
        public static final double kElevatorDrumRadiusMeters = 0.019;
        public static final double kElevatorDrumCircumferenceMeters =
                2.0 * Math.PI * kElevatorDrumRadiusMeters;
        public static final double kElevatorCarriageMassKg = 7.0;

        // Arm motor ids
        public static final int kARM_MOTOR_ID = 61;

        // Arm gains
        public static final double kArmP = 50.0;
        public static final double kArmS = 0.0;
        public static final double kArmV = 0.0;
        public static final double kArmG = 0.1;

        // Arm configuration
        public static final double kArmStatorCurrentLimit = 80.0;
        public static final double kArmSupplyCurrentLimit = 40.0;
        public static final double kArmReduction = 60.0;
        public static final double kArmMinDegrees = 0.0;
        public static final double kArmMaxDegrees = 90.0;
        public static final double kArmCruiseVelocity = 1.0;
        public static final double kArmAcceleration = 4.0;
        public static final double kArmMOI = 0.005; // kg m^2
        public static final double kArmLengthMeters = 0.7;

        // Hand motor ids
        public static final int kHAND_MOTOR_ID = 62;

        // Hand configuration
        public static final double kHandStatorCurrentLimit = 80.0;
        public static final double kHandSupplyCurrentLimit = 40.0;
        public static final double kHandReduction = 3.45;
        public static final double kHandMOI = 0.001; // kg m^2

        // Hand bring-up test values
        public static final double kHandTestVolts = 4.0;
    }

    /**
     * Bring-up test states. Replace these with the real states for this
     * mechanism once the hardware is verified.
     */
    public enum ArmState {
        IDLE,
        TEST_FORWARD,
        TEST_REVERSE
    }

    private final ArmIO io;
    private final ArmIO.ArmInputs inputs = new ArmIO.ArmInputs();

    private ArmState state = ArmState.IDLE;

    private final MechanismLigament2d armLigament = new MechanismLigament2d(
            "arm",
            Units.inchesToMeters(8.0),
            10.854,
            6.0,
            new Color8Bit(52, 235, 137));

    public Arm() {
        this(new ArmIO() {
        });
    }

    public Arm(ArmIO io) {
        this.io = io;
        RobotVisualizer.addArm(armLigament);
    }

    public void setState(ArmState state) {
        this.state = state;
    }

    public ArmState getState() {
        return state;
    }

    public void stop() {
        state = ArmState.IDLE;
        io.stop();
    }

    public void resetEncoder() {
        io.resetEncoder();
    }

    public void requestIdle() {
        setState(ArmState.IDLE);
    }

    public void requestTestForward() {
        setState(ArmState.TEST_FORWARD);
    }

    public void requestTestReverse() {
        setState(ArmState.TEST_REVERSE);
    }

    public double getElevatorPositionMeters() {
        return inputs.elevatorPositionMeters;
    }

    public double getElevatorAppliedVolts() {
        return inputs.elevatorAppliedVolts;
    }

    public boolean areElevatorMotorsConnected() {
        return inputs.elevatorLeadMotorConnected
                && inputs.elevatorFollowerMotorConnected;
    }

    public double getArmPositionDegrees() {
        return inputs.armPositionDegrees;
    }

    public double getArmAppliedVolts() {
        return inputs.armAppliedVolts;
    }

    public boolean isArmMotorConnected() {
        return inputs.armMotorConnected;
    }

    public double getHandVelocityRPM() {
        return inputs.handVelocityRPM;
    }

    public double getHandAppliedVolts() {
        return inputs.handAppliedVolts;
    }

    public boolean isHandMotorConnected() {
        return inputs.handMotorConnected;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        armLigament.setLength(
                Units.inchesToMeters(8.0) + inputs.elevatorPositionMeters);

        io.setElevatorPosition(resolveElevatorTargetPosition(state));
        io.setArmPosition(resolveArmTargetPosition(state));
        io.setHandVoltage(resolveHandTargetVoltage(state));
    }

    private static double resolveElevatorTargetPosition(ArmState state) {
        return switch (state) {
            case IDLE -> ArmConstants.kElevatorMinMeters;
            case TEST_FORWARD -> ArmConstants.kElevatorMaxMeters;
            case TEST_REVERSE -> ArmConstants.kElevatorMinMeters;
        };
    }

    private static double resolveArmTargetPosition(ArmState state) {
        return switch (state) {
            case IDLE -> ArmConstants.kArmMinDegrees;
            case TEST_FORWARD -> ArmConstants.kArmMaxDegrees;
            case TEST_REVERSE -> ArmConstants.kArmMinDegrees;
        };
    }

    private static double resolveHandTargetVoltage(ArmState state) {
        return switch (state) {
            case IDLE -> 0.0;
            case TEST_FORWARD -> ArmConstants.kHandTestVolts;
            case TEST_REVERSE -> -ArmConstants.kHandTestVolts;
        };
    }
}
