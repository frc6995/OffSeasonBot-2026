package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    public class IntakeConstants {
    public static final int kKICKER_MOTOR_ID = 34;
    public static final int kROLLER_LEAD_MOTOR_ID = 30;
    public static final int kROLLER_FOLLOWER_MOTOR_ID = 31;
    public static final int kEXTENSION_LEAD_MOTOR_ID = 32;
    public static final int kEXTENSION_FOLLOWER_MOTOR_ID = 33;

    public static final double kKickerStatorCurrentLimit = 80.0;
    public static final double kKickerSupplyCurrentLimit = 40.0;
    public static final double kKickerReduction = 1.5;

    public static final double kRollerStatorCurrentLimit = 80.0;
    public static final double kRollerSupplyCurrentLimit = 40.0;
    public static final double kRollerReduction = 3.45;

    public static final double kExtensionStatorCurrentLimit = 80.0;
    public static final double kExtensionSupplyCurrentLimit = 40.0;
    public static final double kExtensionReduction = 3.33;

    public static final double kVelMetersPerSecond = 0.5;
    public static final double kAccelMetersPerSecondSquared = 0.5;

    public static final double kExtensionP = 0.5;
    public static final double kExtensionV = 0.5;

    public static final double kExtensionMaxMeters = 0.5;
    public static final double kExtensionMinMeters = 0.0;

    public static final double kDrumCircumferenceMeters = 0.0;

    public static final double kRollerForwardVelocity = 50.0;
    public static final double kKickerForwardVelocity = 50.0;
    }

    public enum States {
        RETRACTED,
        DEPLOYED,
        IDLE,
        AGITATING,
    }

    private final IntakeIO io;
    private final IntakeIO.IntakeInputs inputs = new IntakeIO.IntakeInputs();

    private States intakeState = States.IDLE;

    public Intake() {
    this(new IntakeIO() {});
    }

    public Intake(IntakeIO io) {
    this.io = io;
    }

    public void stop() {
        intakeState = States.IDLE;
    }

    public void setState(States state) {
        intakeState = state;
    }

    public States getState() {
        return intakeState;
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

    io.setKickerVoltage(resolveKickerTargetVelocity(intakeState));
    io.setRollerVoltage(resolveRollerTargetVelocity(intakeState));
    io.setExtensionVoltage(resolveExtensionTargetPosition(intakeState));
  }

    private static double resolveExtensionTargetPosition(States state) {
        return switch (state) {
            case IDLE -> 0.0;
            case RETRACTED -> IntakeConstants.kExtensionMinMeters;
            case DEPLOYED -> IntakeConstants.kExtensionMaxMeters;
            case AGITATING -> 0.0;
            default -> 0.0;
        };
    }

    private static double resolveRollerTargetVelocity(States state) {
        return switch (state) {
            case IDLE -> 0.0;
            case RETRACTED -> 0.0;
            case DEPLOYED -> IntakeConstants.kRollerForwardVelocity;
            case AGITATING -> 0.0;
            default -> 0.0;
        };
    }

    private static double resolveKickerTargetVelocity(States state) {
        return switch (state) {
            case IDLE -> 0.0;
            case RETRACTED -> 0.0;
            case DEPLOYED -> IntakeConstants.kKickerForwardVelocity;
            case AGITATING -> 0.0;
            default -> 0.0;
        };
    }
}