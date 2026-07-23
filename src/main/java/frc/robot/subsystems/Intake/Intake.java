package frc.robot.subsystems.Intake;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    public static final class IntakeConstants {
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

    public static final double kDrumCircumferenceMeters = 1.0;

    public static final double kRollerForwardVolts = 12.0;
    public static final double kKickerForwardVolts = 12.0;

    public static final double kRollerForwardVelocity = kRollerForwardVolts;
    public static final double kKickerForwardVelocity = kKickerForwardVolts;

    public static final double acceleration = 5.0;
    public static final double velocity = 5.0;
    }

    public enum IntakeState {
        RETRACTED,
        DEPLOYED,
        IDLE,
        AGITATING,
    }

    private final IntakeIO io;
    private final IntakeIO.IntakeInputs inputs = new IntakeIO.IntakeInputs();

    private IntakeState intakeState = IntakeState.IDLE;

    public Intake() {
    this(new IntakeIO() {});
    }

    public Intake(IntakeIO io) {
    this.io = io;
    }

    public void stop() {
        intakeState = IntakeState.IDLE;
        io.stop();
    }

    public void setState(IntakeState state) {
        intakeState = state;
    }

    public IntakeState getState() {
        return intakeState;
    }

    public void retract() {
        setState(IntakeState.RETRACTED);
    }

    public void deploy() {
        setState(IntakeState.DEPLOYED);
    }

    public void setIdle() {
        setState(IntakeState.IDLE);
    }

    public void agitate() {
        setState(IntakeState.AGITATING);
    }

    public void resetEncoder() {
        io.resetEncoder();
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

    io.setKickerVoltage(resolveKickerTargetVoltage(intakeState));
    io.setRollerVoltage(resolveRollerTargetVoltage(intakeState));
    io.setExtensionPosition(resolveExtensionTargetPosition(intakeState));
  }

    private static double resolveExtensionTargetPosition(IntakeState state) {
        return switch (state) {
            case IDLE -> IntakeConstants.kExtensionMinMeters;
            case RETRACTED -> IntakeConstants.kExtensionMinMeters;
            case DEPLOYED -> IntakeConstants.kExtensionMaxMeters;
            case AGITATING -> IntakeConstants.kExtensionMinMeters;
        };
    }

    private static double resolveRollerTargetVoltage(IntakeState state) {
        return switch (state) {
            case IDLE -> 0.0;
            case RETRACTED -> 0.0;
            case DEPLOYED -> IntakeConstants.kRollerForwardVolts;
            case AGITATING -> 0.0;
        };
    }

    private static double resolveKickerTargetVoltage(IntakeState state) {
        return switch (state) {
            case IDLE -> 0.0;
            case RETRACTED -> 0.0;
            case DEPLOYED -> IntakeConstants.kKickerForwardVolts;
            case AGITATING -> 0.0;
        };
    }
}
