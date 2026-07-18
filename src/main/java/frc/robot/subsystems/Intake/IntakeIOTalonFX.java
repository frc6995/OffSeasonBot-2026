package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.Intake.Intake.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {
    protected final TalonFX m_rollerLeadMotor
    = new TalonFX(Intake.IntakeConstants.kROLLER_LEAD_MOTOR_ID);
   // = new TalonFX(IntakeConstants.kROLLER_MOTOR_ID, Constants.CanBuses.kUpperCANBus);

   protected final TalonFX m_rollerFollowerMotor 
    = new TalonFX(Intake.IntakeConstants.kROLLER_FOLLOWER_MOTOR_ID);

    protected final TalonFX m_extensionLeadMotor
    = new TalonFX(Intake.IntakeConstants.kEXTENSION_LEAD_MOTOR_ID);
    // = new TalonFX(IntakeConstants.kEXTENSION_LEAD_MOTOR_ID, Constants.CanBuses.kUpperCANBus);

    protected final TalonFX m_extensionFollowerMotor
    = new TalonFX(Intake.IntakeConstants.kEXTENSION_FOLLOWER_MOTOR_ID);
    // = new TalonFX(IntakeConstants.kEXTENSION_FOLLOWER_MOTOR_ID, Constants.CanBuses.kUpperCANBus);

   protected final TalonFX m_kickerMotor
    = new TalonFX(Intake.IntakeConstants.kKICKER_MOTOR_ID);
   // = new TalonFX(IntakeConstants.kKICKER_MOTOR_ID, Constants.CanBuses.kUpperCANBus);

    private final VoltageOut m_rollerRequest = new VoltageOut(0);
    private final VoltageOut m_kickerRequest = new VoltageOut(0);

    protected final MotionMagicVoltage m_extensionRequest =
    new MotionMagicVoltage(0.0).withEnableFOC(true);

    private final StatusSignal<Voltage> m_rollerAppliedVoltage = m_rollerLeadMotor.getMotorVoltage();
    private final StatusSignal<Current> m_rollerStatorCurrent = m_rollerLeadMotor.getStatorCurrent();
    private final StatusSignal<Current> m_rollerSupplyCurrent = m_rollerLeadMotor.getSupplyCurrent();
    private final StatusSignal<Voltage> m_rollerFollowerAppliedVoltage = m_rollerFollowerMotor.getMotorVoltage();

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
        kickConfig.MotorOutput =
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);
        kickConfig.CurrentLimits =
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(IntakeConstants.kKickerStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(IntakeConstants.kKickerSupplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true);
        kickConfig.Feedback =
            new FeedbackConfigs().withSensorToMechanismRatio(IntakeConstants.kKickerReduction);
        m_kickerMotor.getConfigurator().apply(kickConfig);
    }

    private void configureRollerMotors() {
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput =
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);
        rollerConfig.CurrentLimits =
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(IntakeConstants.kRollerStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(IntakeConstants.kRollerSupplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true);
        rollerConfig.Feedback =
            new FeedbackConfigs().withSensorToMechanismRatio(IntakeConstants.kRollerReduction);
        m_rollerLeadMotor.getConfigurator().apply(rollerConfig);
        m_rollerFollowerMotor.getConfigurator().apply(rollerConfig);

        m_rollerFollowerMotor.setControl(new Follower(m_rollerLeadMotor.getDeviceID(), MotorAlignmentValue.Opposed));
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
        m_extensionLeadMotor.getConfigurator().apply(extensionConfig);
        m_extensionFollowerMotor.getConfigurator().apply(extensionConfig);

        m_extensionFollowerMotor.setControl(new Follower(m_extensionLeadMotor.getDeviceID(), MotorAlignmentValue.Opposed));
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

        inputs.rollerAppliedVolts = m_rollerAppliedVoltage.getValueAsDouble();
        inputs.rollerStatorCurrentAmps = m_rollerStatorCurrent.getValueAsDouble();
        inputs.rollerSupplyCurrentAmps = m_rollerSupplyCurrent.getValueAsDouble();
        
        inputs.extensionAppliedVolts = m_extensionAppliedVoltage.getValueAsDouble();
        inputs.extensionStatorCurrentAmps = m_extensionStatorCurrent.getValueAsDouble();
        inputs.extensionSupplyCurrentAmps = m_extensionSupplyCurrent.getValueAsDouble();

        inputs.kickerAppliedVolts = m_kickerAppliedVoltage.getValueAsDouble();
        inputs.kickerStatorCurrentAmps = m_kickerStatorCurrent.getValueAsDouble();
        inputs.kickerSupplyCurrentAmps = m_kickerSupplyCurrent.getValueAsDouble();
    }

    @Override
    public void setKickerVoltage(double voltage) {
        m_kickerMotor.setControl(m_kickerRequest.withOutput(voltage));
    }

    @Override
    public void setRollerVoltage(double voltage) {
        m_rollerLeadMotor.setControl(m_rollerRequest.withOutput(voltage));
    }

    @Override
    public void setExtensionPosition(double positionMeters) {
        m_extensionLeadMotor.setControl(m_extensionRequest.withPosition(positionMeters));
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
