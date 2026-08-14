package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm.ArmConstants;
import frc.robot.util.CtreUtil;

public class ArmIOTalonFX implements ArmIO {

    protected final TalonFX m_elevatorLeadMotor =
            new TalonFX(ArmConstants.kELEVATOR_LEAD_MOTOR_ID, Constants.CANBuses.UpperBus);
    protected final TalonFX m_elevatorFollowerMotor =
            new TalonFX(ArmConstants.kELEVATOR_FOLLOWER_MOTOR_ID, Constants.CANBuses.UpperBus);

    protected final TalonFX m_armMotor =
            new TalonFX(ArmConstants.kARM_MOTOR_ID, Constants.CANBuses.UpperBus);

    protected final TalonFX m_handMotor =
            new TalonFX(ArmConstants.kHAND_MOTOR_ID, Constants.CANBuses.UpperBus);

    protected final MotionMagicVoltage m_elevatorRequest =
            new MotionMagicVoltage(0.0).withEnableFOC(true);
    protected final MotionMagicVoltage m_armRequest =
            new MotionMagicVoltage(0.0).withEnableFOC(true);
    protected final VoltageOut m_handRequest =
            new VoltageOut(0.0).withEnableFOC(true);

    private final StatusSignal<Angle> m_elevatorPosition = m_elevatorLeadMotor.getPosition();
    private final StatusSignal<Voltage> m_elevatorAppliedVoltage =
            m_elevatorLeadMotor.getMotorVoltage();
    private final StatusSignal<Current> m_elevatorStatorCurrent =
            m_elevatorLeadMotor.getStatorCurrent();
    private final StatusSignal<Current> m_elevatorSupplyCurrent =
            m_elevatorLeadMotor.getSupplyCurrent();
    private final StatusSignal<Voltage> m_elevatorFollowerAppliedVoltage =
            m_elevatorFollowerMotor.getMotorVoltage();

    private final StatusSignal<Angle> m_armPosition = m_armMotor.getPosition();
    private final StatusSignal<Voltage> m_armAppliedVoltage = m_armMotor.getMotorVoltage();
    private final StatusSignal<Current> m_armStatorCurrent = m_armMotor.getStatorCurrent();
    private final StatusSignal<Current> m_armSupplyCurrent = m_armMotor.getSupplyCurrent();

    private final StatusSignal<AngularVelocity> m_handVelocity = m_handMotor.getVelocity();
    private final StatusSignal<Voltage> m_handAppliedVoltage = m_handMotor.getMotorVoltage();
    private final StatusSignal<Current> m_handStatorCurrent = m_handMotor.getStatorCurrent();
    private final StatusSignal<Current> m_handSupplyCurrent = m_handMotor.getSupplyCurrent();

    public ArmIOTalonFX() {
        configureMotors();
    }

    protected void configureMotors() {
        configureElevatorMotors();
        configureArmMotors();
        configureHandMotors();
    }

    private void configureElevatorMotors() {
        TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
        elevatorConfig.MotorOutput = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive);
        elevatorConfig.CurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(ArmConstants.kElevatorStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(ArmConstants.kElevatorSupplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true);
        elevatorConfig.Feedback = new FeedbackConfigs()
                .withSensorToMechanismRatio(ArmConstants.kElevatorReduction);
        elevatorConfig.Slot0 = new Slot0Configs()
                .withKP(ArmConstants.kElevatorP)
                .withKS(ArmConstants.kElevatorS)
                .withKV(ArmConstants.kElevatorV)
                .withKG(ArmConstants.kElevatorG)
                .withGravityType(GravityTypeValue.Elevator_Static);
        elevatorConfig.MotionMagic = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(ArmConstants.kElevatorCruiseVelocity)
                .withMotionMagicAcceleration(ArmConstants.kElevatorAcceleration);
        elevatorConfig.SoftwareLimitSwitch = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(
                        elevatorMetersToRotations(ArmConstants.kElevatorMaxMeters))
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(
                        elevatorMetersToRotations(ArmConstants.kElevatorMinMeters));

        CtreUtil.reportIfNotOk(
                "configure elevator",
                m_elevatorLeadMotor.getConfigurator().apply(elevatorConfig));
        CtreUtil.reportIfNotOk(
                "configure elevator follower 1",
                m_elevatorFollowerMotor.getConfigurator().apply(elevatorConfig));
        m_elevatorFollowerMotor.setControl(
                new Follower(m_elevatorLeadMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    private void configureArmMotors() {
        TalonFXConfiguration armConfig = new TalonFXConfiguration();
        armConfig.MotorOutput = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive);
        armConfig.CurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(ArmConstants.kArmStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(ArmConstants.kArmSupplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true);
        armConfig.Feedback = new FeedbackConfigs()
                .withSensorToMechanismRatio(ArmConstants.kArmReduction);
        armConfig.Slot0 = new Slot0Configs()
                .withKP(ArmConstants.kArmP)
                .withKS(ArmConstants.kArmS)
                .withKV(ArmConstants.kArmV)
                .withKG(ArmConstants.kArmG)
                .withGravityType(GravityTypeValue.Arm_Cosine);
        armConfig.MotionMagic = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(ArmConstants.kArmCruiseVelocity)
                .withMotionMagicAcceleration(ArmConstants.kArmAcceleration);
        armConfig.SoftwareLimitSwitch = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(
                        armDegreesToRotations(ArmConstants.kArmMaxDegrees))
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(
                        armDegreesToRotations(ArmConstants.kArmMinDegrees));

        CtreUtil.reportIfNotOk(
                "configure arm",
                m_armMotor.getConfigurator().apply(armConfig));
    }

    private void configureHandMotors() {
        TalonFXConfiguration handConfig = new TalonFXConfiguration();
        handConfig.MotorOutput = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.CounterClockwise_Positive);
        handConfig.CurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(ArmConstants.kHandStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(ArmConstants.kHandSupplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true);
        handConfig.Feedback = new FeedbackConfigs()
                .withSensorToMechanismRatio(ArmConstants.kHandReduction);

        CtreUtil.reportIfNotOk(
                "configure hand",
                m_handMotor.getConfigurator().apply(handConfig));
    }

    @Override
    public void updateInputs(ArmInputs inputs) {
        inputs.elevatorLeadMotorConnected =
                BaseStatusSignal.refreshAll(
                        m_elevatorPosition,
                        m_elevatorAppliedVoltage,
                        m_elevatorStatorCurrent,
                        m_elevatorSupplyCurrent)
                        .isOK();
        inputs.elevatorFollowerMotorConnected =
                BaseStatusSignal.refreshAll(m_elevatorFollowerAppliedVoltage).isOK();

        inputs.elevatorPositionMeters =
                elevatorRotationsToMeters(m_elevatorPosition.getValueAsDouble());
        inputs.elevatorAppliedVolts = m_elevatorAppliedVoltage.getValueAsDouble();
        inputs.elevatorStatorCurrentAmps = m_elevatorStatorCurrent.getValueAsDouble();
        inputs.elevatorSupplyCurrentAmps = m_elevatorSupplyCurrent.getValueAsDouble();

        inputs.armMotorConnected =
                BaseStatusSignal.refreshAll(
                        m_armPosition,
                        m_armAppliedVoltage,
                        m_armStatorCurrent,
                        m_armSupplyCurrent)
                        .isOK();

        inputs.armPositionDegrees =
                armRotationsToDegrees(m_armPosition.getValueAsDouble());
        inputs.armAppliedVolts = m_armAppliedVoltage.getValueAsDouble();
        inputs.armStatorCurrentAmps = m_armStatorCurrent.getValueAsDouble();
        inputs.armSupplyCurrentAmps = m_armSupplyCurrent.getValueAsDouble();

        inputs.handMotorConnected =
                BaseStatusSignal.refreshAll(
                        m_handVelocity,
                        m_handAppliedVoltage,
                        m_handStatorCurrent,
                        m_handSupplyCurrent)
                        .isOK();

        inputs.handVelocityRPM = m_handVelocity.getValueAsDouble() * 60.0;
        inputs.handAppliedVolts = m_handAppliedVoltage.getValueAsDouble();
        inputs.handStatorCurrentAmps = m_handStatorCurrent.getValueAsDouble();
        inputs.handSupplyCurrentAmps = m_handSupplyCurrent.getValueAsDouble();
    }

    @Override
    public void setElevatorPosition(double positionMeters) {
        m_elevatorLeadMotor.setControl(
                m_elevatorRequest.withPosition(elevatorMetersToRotations(positionMeters)));
    }

    @Override
    public void setArmPosition(double positionDegrees) {
        m_armMotor.setControl(
                m_armRequest.withPosition(armDegreesToRotations(positionDegrees)));
    }

    @Override
    public void setHandVoltage(double volts) {
        m_handMotor.setControl(m_handRequest.withOutput(volts));
    }

    @Override
    public void resetEncoder() {
        m_elevatorLeadMotor.setPosition(0.0);
        m_elevatorFollowerMotor.setPosition(0.0);
        m_armMotor.setPosition(0.0);
    }

    @Override
    public void stop() {
        m_elevatorLeadMotor.stopMotor();
        m_armMotor.stopMotor();
        m_handMotor.stopMotor();
    }

    protected static double elevatorMetersToRotations(double meters) {
        return meters / ArmConstants.kElevatorDrumCircumferenceMeters;
    }

    protected static double elevatorRotationsToMeters(double rotations) {
        return rotations * ArmConstants.kElevatorDrumCircumferenceMeters;
    }

    protected static double armDegreesToRotations(double degrees) {
        return degrees / 360.0;
    }

    protected static double armRotationsToDegrees(double rotations) {
        return rotations * 360.0;
    }
}
