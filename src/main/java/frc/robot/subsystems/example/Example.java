package frc.robot.subsystems.example;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotVisualizer;

public class Example extends SubsystemBase {

    public static final class ExampleConstants {
        // IntakePivot motor ids
        public static final int kINTAKE_PIVOT_MOTOR_ID = 58;

        // IntakePivot gains
        public static final double kIntakePivotP = 20.0;
        public static final double kIntakePivotS = 0.25;
        public static final double kIntakePivotV = 0.396;
        public static final double kIntakePivotG = 0.0;

        // IntakePivot configuration
        public static final double kIntakePivotStatorCurrentLimit = 80.0;
        public static final double kIntakePivotSupplyCurrentLimit = 40.0;
        public static final double kIntakePivotReduction = 50.0;
        public static final double kIntakePivotMinDegrees = 0.0;
        public static final double kIntakePivotMaxDegrees = 90.0;
        public static final double kIntakePivotCruiseVelocity = 1.0;
        public static final double kIntakePivotAcceleration = 2.0;
        public static final double kIntakePivotMOI = 0.001; // kg m^2
        public static final double kIntakePivotLengthMeters = 0.3;

        // IntakeRoller motor ids
        public static final int kINTAKE_ROLLER_MOTOR_ID = 59;

        // IntakeRoller gains
        public static final double kIntakeRollerP = 0.1;
        public static final double kIntakeRollerS = 0.0;
        public static final double kIntakeRollerV = 0.1;

        // IntakeRoller configuration
        public static final double kIntakeRollerStatorCurrentLimit = 80.0;
        public static final double kIntakeRollerSupplyCurrentLimit = 40.0;
        public static final double kIntakeRollerReduction = 1.0;
        public static final double kIntakeRollerMOI = 0.001; // kg m^2

        // IntakeRoller bring-up test values
        public static final double kIntakeRollerTestVelocityRPM = 1000.0;
    }

    /**
     * Bring-up test states. Replace these with the real states for this
     * mechanism once the hardware is verified.
     */
    public enum ExampleState {
        IDLE,
        TEST_FORWARD,
        TEST_REVERSE
    }

    private final ExampleIO io;
    private final ExampleIO.ExampleInputs inputs = new ExampleIO.ExampleInputs();

    private ExampleState state = ExampleState.IDLE;

    private final MechanismLigament2d exampleLigament = new MechanismLigament2d(
            "example",
            Units.inchesToMeters(8.0),
            0.0,
            6.0,
            new Color8Bit(52, 235, 137));

    public Example() {
        this(new ExampleIO() {
        });
    }

    public Example(ExampleIO io) {
        this.io = io;
        RobotVisualizer.addExample(exampleLigament);
    }

    public void setState(ExampleState state) {
        this.state = state;
    }

    public ExampleState getState() {
        return state;
    }

    public void stop() {
        state = ExampleState.IDLE;
        io.stop();
    }

    public void resetEncoder() {
        io.resetEncoder();
    }

    public void requestIdle() {
        setState(ExampleState.IDLE);
    }

    public void requestTestForward() {
        setState(ExampleState.TEST_FORWARD);
    }

    public void requestTestReverse() {
        setState(ExampleState.TEST_REVERSE);
    }

    public double getIntakePivotPositionDegrees() {
        return inputs.intakePivotPositionDegrees;
    }

    public double getIntakePivotAppliedVolts() {
        return inputs.intakePivotAppliedVolts;
    }

    public boolean isIntakePivotMotorConnected() {
        return inputs.intakePivotMotorConnected;
    }

    public double getIntakeRollerVelocityRPM() {
        return inputs.intakeRollerVelocityRPM;
    }

    public double getIntakeRollerAppliedVolts() {
        return inputs.intakeRollerAppliedVolts;
    }

    public boolean isIntakeRollerMotorConnected() {
        return inputs.intakeRollerMotorConnected;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        exampleLigament.setAngle(inputs.intakePivotPositionDegrees);

        io.setIntakePivotPosition(resolveIntakePivotTargetPosition(state));
        io.setIntakeRollerVelocity(resolveIntakeRollerTargetVelocity(state));
    }

    private static double resolveIntakePivotTargetPosition(ExampleState state) {
        return switch (state) {
            case IDLE -> ExampleConstants.kIntakePivotMinDegrees;
            case TEST_FORWARD -> ExampleConstants.kIntakePivotMaxDegrees;
            case TEST_REVERSE -> ExampleConstants.kIntakePivotMinDegrees;
        };
    }

    private static double resolveIntakeRollerTargetVelocity(ExampleState state) {
        return switch (state) {
            case IDLE -> 0.0;
            case TEST_FORWARD -> ExampleConstants.kIntakeRollerTestVelocityRPM;
            case TEST_REVERSE -> -ExampleConstants.kIntakeRollerTestVelocityRPM;
        };
    }
}
