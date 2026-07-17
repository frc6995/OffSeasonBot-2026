
package frc.robot.subsystems.dyerotor;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
// import frc.robot.Constants;
// import frc.robot.util.CtreUtil;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.dyerotor.DyeRotor.DyeRotorConstants;

public class DyeRotorIOTalonFX implements DyeRotorIO {
    protected final TalonFX m_spinMotor =
        new TalonFX(DyeRotorConstants.kSpinMotorCANID, Constants.CANBuses.LowerBus);
    protected final TalonFX m_indexerLead =
        new TalonFX(DyeRotorConstants.kLeadIndexMotorCANID, Constants.CANBuses.LowerBus);
    protected final TalonFX m_indexerFollow =
        new TalonFX(DyeRotorConstants.kFollowIndexMotorCANID, Constants.CANBuses.LowerBus);

    // private final VoltageOut m_indexerRequest = new VoltageOut(0);
    private final VelocityVoltage m_indexerRequest = new VelocityVoltage(0);
    private final VelocityVoltage m_spindexerRequest = new VelocityVoltage(0);

    final StatusSignal<AngularVelocity> m_spinVelocity = m_spinMotor.getVelocity();
    final StatusSignal<Voltage> m_spinVoltage = m_spinMotor.getMotorVoltage();
    final StatusSignal<Current> m_spinSupCurrent = m_spinMotor.getSupplyCurrent();
    final StatusSignal<Current> m_spinStatCurrent = m_spinMotor.getStatorCurrent();

    final StatusSignal<AngularVelocity> m_indexVelocity = m_indexerLead.getVelocity();
    final StatusSignal<Voltage> m_indexVoltage = m_indexerLead.getMotorVoltage();
    final StatusSignal<Current> m_indexSupCurrent = m_indexerLead.getSupplyCurrent();
    final StatusSignal<Current> m_indexStatCurrent = m_indexerLead.getStatorCurrent();

    
    public DyeRotorIOTalonFX() {
        configureMotors();
    }

    protected void configureMotors() {
        configureSpinMotor();
        configureIndexMotors();
    }
    
    private void configureSpinMotor() {
        TalonFXConfiguration spinConfig = new TalonFXConfiguration();
        spinConfig.MotorOutput =
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive);
        spinConfig.CurrentLimits =
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(DyeRotorConstants.kSpinStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(DyeRotorConstants.kSpinSupplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true);
        spinConfig.Feedback =
            new FeedbackConfigs().withSensorToMechanismRatio(DyeRotorConstants.kSpinReduction);
        spinConfig.Slot0 =
            new Slot0Configs()
                .withKP(DyeRotorConstants.kSpinKP)
                .withKS(DyeRotorConstants.kSpinKS)
                .withKV(DyeRotorConstants.kSpinKV);
        m_spinMotor.getConfigurator().apply(spinConfig);
    }

    private void configureIndexMotors() {
        TalonFXConfiguration indexConfig = new TalonFXConfiguration();
        indexConfig.MotorOutput =
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive);
        indexConfig.CurrentLimits =
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(DyeRotorConstants.kIndexStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(DyeRotorConstants.kIndexSupplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true);
        indexConfig.Feedback =
            new FeedbackConfigs().withSensorToMechanismRatio(DyeRotorConstants.kIndexReduction);
        indexConfig.Slot0 =
            new Slot0Configs()
                .withKP(DyeRotorConstants.kIndexKP)
                .withKS(DyeRotorConstants.kIndexKS)
                .withKV(DyeRotorConstants.kIndexKV);
        m_indexerLead.getConfigurator().apply(indexConfig);
        m_indexerFollow.getConfigurator().apply(indexConfig);

        m_indexerFollow.setControl(new Follower(m_indexerLead.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    @Override
    public void updateInputs(DyeRotorInputs inputs) {
        BaseStatusSignal.refreshAll(
            m_spinVelocity, m_spinVoltage, m_spinSupCurrent, m_spinStatCurrent, 
            m_indexVelocity, m_indexVoltage, m_indexSupCurrent, m_indexStatCurrent);

        inputs.spinVelocityRPM = m_spinVelocity.getValueAsDouble() * 60.0;
        inputs.spinAppliedVolts = m_spinVoltage.getValueAsDouble();
        inputs.spinStatorCurrentAmps = m_spinStatCurrent.getValueAsDouble();
        inputs.spinSupplyCurrentAmps = m_spinSupCurrent.getValueAsDouble();
        inputs.spinMotorConnected = m_spinMotor.isConnected();

        inputs.indexVelocityRPM = m_indexVelocity.getValueAsDouble() * 60.0;
        inputs.indexAppliedVolts = m_indexVoltage.getValueAsDouble();
        inputs.indexStatorCurrentAmps = m_indexStatCurrent.getValueAsDouble();
        inputs.indexSupplyCurrentAmps = m_indexSupCurrent.getValueAsDouble();
        inputs.indexLeadMotorConnected = m_indexerLead.isConnected();
        inputs.indexFollowerMotorConnected = m_indexerFollow.isConnected();
    }

    @Override

    // Apparently this expects rotations per second
    public void setSpinVelocity(double velocityRPM) {
        double velocityRPS = velocityRPM / 60.0;
        m_spinMotor.setControl(m_spindexerRequest.withVelocity(velocityRPS));
    }

    // @Override
    // public void setIndexVoltage(double voltage) {
    //     m_indexerLead.setControl(m_indexerRequest.withOutput(voltage));
    // }
    @Override
    public void setIndexVelocity(double velocityRPM) {
        double velocityRPS = velocityRPM / 60.0;
        m_indexerLead.setControl(m_indexerRequest.withVelocity(velocityRPS));
    }

    @Override
    public void stop() {
        m_spinMotor.stopMotor();
        m_indexerLead.stopMotor();
    }
    
    // protected static double mechanismRotationsToSpinMotorRotations(double rotations) {
    //     return rotations * DyeRotorConstants.kSpinReduction;
    // }
    // protected static double motorRotationsToSpinMechanismRotations(double rotations) {
    //     return rotations / DyeRotorConstants.kSpinReduction;
    // }
    // protected static double mechanismRotationsToIndexMotorRotations(double rotations) {
    //     return rotations * DyeRotorConstants.kIndexReduction;
    // }
}