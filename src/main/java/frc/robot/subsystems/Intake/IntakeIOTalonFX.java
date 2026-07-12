package frc.robot.subsystems.Intake;

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

        inputs.rollerAppliedVolts = m_rollerLeadMotor.getMotorVoltage().refresh().getValueAsDouble();
        inputs.rollerStatorCurrentAmps = m_rollerLeadMotor.getStatorCurrent().refresh().getValueAsDouble();
        inputs.rollerSupplyCurrentAmps = m_rollerLeadMotor.getSupplyCurrent().refresh().getValueAsDouble();
        
        inputs.extensionAppliedVolts = m_extensionLeadMotor.getMotorVoltage().refresh().getValueAsDouble();
        inputs.extensionStatorCurrentAmps = m_extensionLeadMotor.getStatorCurrent().refresh().getValueAsDouble();
        inputs.extensionSupplyCurrentAmps = m_extensionLeadMotor.getSupplyCurrent().refresh().getValueAsDouble();

        inputs.kickerAppliedVolts = m_kickerMotor.getMotorVoltage().refresh().getValueAsDouble();
        inputs.kickerStatorCurrentAmps = m_kickerMotor.getStatorCurrent().refresh().getValueAsDouble();
        inputs.kickerSupplyCurrentAmps = m_kickerMotor.getSupplyCurrent().refresh().getValueAsDouble();
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
    public void setExtensionVoltage(double position) {
        m_extensionLeadMotor.setControl(m_extensionRequest.withPosition(position));
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
