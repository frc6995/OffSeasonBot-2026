package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake.IntakeConstants;
import frc.robot.util.TorqueCurrentLimiter;

public class IntakeIOTalonFX implements IntakeIO {
    protected final TalonFX m_rollerLeadMotor
    = new TalonFX(IntakeConstants.kROLLER_LEAD_MOTOR_ID, Constants.CANBuses.UpperBus);

    protected final TalonFX m_rollerFollowerMotor 
    = new TalonFX(Intake.IntakeConstants.kROLLER_FOLLOWER_MOTOR_ID, Constants.CANBuses.UpperBus);

    protected final TalonFX m_extensionLeadMotor
    = new TalonFX(Intake.IntakeConstants.kEXTENSION_LEAD_MOTOR_ID, Constants.CANBuses.UpperBus);

    protected final TalonFX m_extensionFollowerMotor
    = new TalonFX(Intake.IntakeConstants.kEXTENSION_FOLLOWER_MOTOR_ID, Constants.CANBuses.UpperBus);

    protected final TalonFX m_kickerMotor
    = new TalonFX(Intake.IntakeConstants.kKICKER_MOTOR_ID, Constants.CANBuses.UpperBus);

    protected VelocityTorqueCurrentFOC m_rollerVelocityRequest = new VelocityTorqueCurrentFOC(0);
    protected VelocityTorqueCurrentFOC m_kickerVelocityRequest = new VelocityTorqueCurrentFOC(0);

    protected final MotionMagicTorqueCurrentFOC m_extensionRequest =
    new MotionMagicTorqueCurrentFOC(0.0);

    // Each caps closed-loop torque current for its motor(s); a call is only ever
    // sent over CAN when the requested limit actually changes.
    private final TorqueCurrentLimiter m_extensionTorqueCurrentLimiter = new TorqueCurrentLimiter(m_extensionLeadMotor);
    private final TorqueCurrentLimiter m_rollerTorqueCurrentLimiter = new TorqueCurrentLimiter(m_rollerLeadMotor);
    private final TorqueCurrentLimiter m_kickerTorqueCurrentLimiter = new TorqueCurrentLimiter(m_kickerMotor);

    private final StatusSignal<Voltage> m_rollerAppliedVoltage = m_rollerLeadMotor.getMotorVoltage();
    private final StatusSignal<Current> m_rollerStatorCurrent = m_rollerLeadMotor.getStatorCurrent();
    private final StatusSignal<Current> m_rollerSupplyCurrent = m_rollerLeadMotor.getSupplyCurrent();
    private final StatusSignal<Voltage> m_rollerFollowerAppliedVoltage = m_rollerFollowerMotor.getMotorVoltage();

    private final StatusSignal<Angle> m_extensionPosition = m_extensionLeadMotor.getPosition();
    private final StatusSignal<Voltage> m_extensionAppliedVoltage = m_extensionLeadMotor.getMotorVoltage();
    private final StatusSignal<Current> m_extensionStatorCurrent = m_extensionLeadMotor.getStatorCurrent();
    private final StatusSignal<Current> m_extensionSupplyCurrent = m_extensionLeadMotor.getSupplyCurrent();
    private final StatusSignal<Voltage> m_extensionFollowerAppliedVoltage = m_extensionFollowerMotor.getMotorVoltage();

    private final StatusSignal<Voltage> m_kickerAppliedVoltage = m_kickerMotor.getMotorVoltage();
    private final StatusSignal<Current> m_kickerStatorCurrent = m_kickerMotor.getStatorCurrent();
    private final StatusSignal<Current> m_kickerSupplyCurrent = m_kickerMotor.getSupplyCurrent();

    public IntakeIOTalonFX() {
        configureMotors();
    }

    protected void configureMotors() {
        configureKickMotor();
        configureRollerMotors();
        configureExtensionMotors();
    }

    private void configureKickMotor() {
        TalonFXConfiguration kickConfig = new TalonFXConfiguration();
        kickConfig.MotorOutput = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.Clockwise_Positive);
        kickConfig.CurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(IntakeConstants.kKickerStatorCurrentLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(IntakeConstants.kKickerSupplyCurrentLimit)
            .withSupplyCurrentLimitEnable(true);
        kickConfig.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(IntakeConstants.kKickerReduction);
        kickConfig.Slot0 = new Slot0Configs()
            .withKP(IntakeConstants.kKickerP)
            .withKS(IntakeConstants.kKickerS)
            .withKV(IntakeConstants.kKickerV);
        kickConfig.Voltage = new VoltageConfigs()
            .withPeakForwardVoltage(IntakeConstants.kKickerMaxVoltage)
            .withPeakReverseVoltage(IntakeConstants.kKickerMinVoltage);
        // Closed-loop output is torque current (see m_kickerVelocityRequest below);
        // m_kickerTorqueCurrentLimiter owns the actual peak-current values below.
        kickConfig.TorqueCurrent = new TorqueCurrentConfigs();
        // CtreUtil.reportIfNotOk("configure example",
        // m_exMotor.getConfigurator().apply(config));
        m_kickerMotor.getConfigurator().apply(kickConfig);
        m_kickerTorqueCurrentLimiter.setPeakTorqueCurrentAmps(IntakeConstants.kKickerNormalPeakTorqueCurrentAmps);
    }

    private void configureRollerMotors() {
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.CounterClockwise_Positive);
        rollerConfig.CurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(IntakeConstants.kRollerStatorCurrentLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(IntakeConstants.kRollerSupplyCurrentLimit)
            .withSupplyCurrentLimitEnable(true);
        rollerConfig.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(IntakeConstants.kRollerReduction);
        m_rollerFollowerMotor.setControl(new Follower(m_rollerLeadMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        rollerConfig.Slot0 = new Slot0Configs()
            .withKP(IntakeConstants.kRollerP)
            .withKS(IntakeConstants.kRollerS)
            .withKV(IntakeConstants.kRollerV);
        rollerConfig.Voltage = new VoltageConfigs()
            .withPeakForwardVoltage(IntakeConstants.kRollerMaxVoltage)
            .withPeakReverseVoltage(IntakeConstants.kRollerMinVoltage);
        // Closed-loop output is torque current (see m_rollerVelocityRequest above);
        // m_rollerTorqueCurrentLimiter owns the actual peak-current values below.
        rollerConfig.TorqueCurrent = new TorqueCurrentConfigs();
        // CtreUtil.reportIfNotOk("configure example",
        // m_exMotor.getConfigurator().apply(config));
        m_rollerLeadMotor.getConfigurator().apply(rollerConfig);
        m_rollerTorqueCurrentLimiter.setPeakTorqueCurrentAmps(IntakeConstants.kRollerNormalPeakTorqueCurrentAmps);
    }

    private void configureExtensionMotors() {
         TalonFXConfiguration extensionConfig = new TalonFXConfiguration();
        extensionConfig.MotorOutput =
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);

        extensionConfig.CurrentLimits =
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(IntakeConstants.kExtensionStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(IntakeConstants.kExtensionSupplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true);
        extensionConfig.Feedback =
            new FeedbackConfigs().withSensorToMechanismRatio(IntakeConstants.kExtensionReduction);

        // Closed-loop output is torque current (see m_extensionRequest below);
        // m_extensionTorqueCurrentLimiter owns the actual peak-current values below
        // (and lowers them further while agitating - see setExtensionTorqueCurrentLimit()).
        extensionConfig.TorqueCurrent = new TorqueCurrentConfigs();

        extensionConfig.MotionMagic.withMotionMagicAcceleration(IntakeConstants.acceleration)
             .withMotionMagicCruiseVelocity(IntakeConstants.velocity);

        extensionConfig.Slot0
        .withKP(IntakeConstants.kExtensionP)
        .withKV(IntakeConstants.kExtensionV);

        m_extensionLeadMotor.getConfigurator().apply(extensionConfig);
        m_extensionFollowerMotor.getConfigurator().apply(extensionConfig);
        m_extensionFollowerMotor.setControl(new Follower(m_extensionLeadMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        m_extensionTorqueCurrentLimiter.setPeakTorqueCurrentAmps(IntakeConstants.kExtensionNormalPeakTorqueCurrentAmps);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {

        inputs.rollerLeadMotorConnected =
            BaseStatusSignal.refreshAll(
                m_rollerAppliedVoltage,
                m_rollerStatorCurrent,
                m_rollerSupplyCurrent)
                .isOK();
        inputs.rollerFollowerMotorConnected =
            BaseStatusSignal.refreshAll(m_rollerFollowerAppliedVoltage).isOK();

        inputs.extensionLeadMotorConnected =
            BaseStatusSignal.refreshAll(
                m_extensionPosition,
                m_extensionAppliedVoltage,
                m_extensionStatorCurrent,
                m_extensionSupplyCurrent)
                .isOK();
        inputs.extensionFollowerMotorConnected =
            BaseStatusSignal.refreshAll(m_extensionFollowerAppliedVoltage).isOK();

        
            
        inputs.kickerMotorConnected =
            BaseStatusSignal.refreshAll(
                m_kickerAppliedVoltage,
                m_kickerStatorCurrent,
                m_kickerSupplyCurrent)
                .isOK();

        inputs.rollerVelocityRPM = m_rollerLeadMotor.getVelocity().getValueAsDouble() * 60;
        inputs.rollerAppliedVolts = m_rollerAppliedVoltage.getValueAsDouble();
        inputs.rollerStatorCurrentAmps = m_rollerStatorCurrent.getValueAsDouble();
        inputs.rollerSupplyCurrentAmps = m_rollerSupplyCurrent.getValueAsDouble();
        
        inputs.extensionPositionMeters = mechanismRotationsToMeters(m_extensionPosition.getValueAsDouble());
        inputs.extensionAppliedVolts = m_extensionAppliedVoltage.getValueAsDouble();
        inputs.extensionStatorCurrentAmps = m_extensionStatorCurrent.getValueAsDouble();
        inputs.extensionSupplyCurrentAmps = m_extensionSupplyCurrent.getValueAsDouble();

        inputs.rollerVelocityRPM = m_rollerLeadMotor.getVelocity().getValueAsDouble() * 60;
        inputs.kickerAppliedVolts = m_kickerAppliedVoltage.getValueAsDouble();
        inputs.kickerStatorCurrentAmps = m_kickerStatorCurrent.getValueAsDouble();
        inputs.kickerSupplyCurrentAmps = m_kickerSupplyCurrent.getValueAsDouble();
    }

    @Override
    public void setKickerVelocity(double velocityRPM) {
        m_kickerMotor.setControl(m_kickerVelocityRequest.withVelocity(velocityRPM / 60.0));
    }

    @Override
    public void setRollerVelocity(double velocityRPM) {
        m_rollerLeadMotor.setControl(m_rollerVelocityRequest.withVelocity(velocityRPM / 60.0));
    }

    @Override
    public void setExtensionPosition(double positionMeters) {
        m_extensionLeadMotor.setControl(m_extensionRequest
        .withPosition(metersToMechanismRotations(positionMeters)));
    }

    @Override
    public void setExtensionTorqueCurrentLimit(double peakTorqueCurrentAmps) {
        m_extensionTorqueCurrentLimiter.setPeakTorqueCurrentAmps(peakTorqueCurrentAmps);
    }

    @Override
    public void setRollerTorqueCurrentLimit(double peakTorqueCurrentAmps) {
        m_rollerTorqueCurrentLimiter.setPeakTorqueCurrentAmps(peakTorqueCurrentAmps);
    }

    @Override
    public void setKickerTorqueCurrentLimit(double peakTorqueCurrentAmps) {
        m_kickerTorqueCurrentLimiter.setPeakTorqueCurrentAmps(peakTorqueCurrentAmps);
    }

    public double getExtensionPosition() {
        return m_extensionLeadMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void resetEncoder() {
        m_extensionLeadMotor.setPosition(0.0);
        m_extensionFollowerMotor.setPosition(0.0);
    }


    protected static double metersToMechanismRotations(double meters) {
        return meters / IntakeConstants.kDrumCircumferenceMeters;
    }

    protected static double mechanismRotationsToMeters(double rotations) {
        return rotations * IntakeConstants.kDrumCircumferenceMeters;
    }

    protected static double metersToMotorRotations(double meters) {
        return metersToMechanismRotations(meters) * IntakeConstants.kExtensionReduction;
    }

    @Override
    public void stop() {
        m_extensionLeadMotor.stopMotor();
        m_rollerLeadMotor.stopMotor();
        m_kickerMotor.stopMotor();
    }
}