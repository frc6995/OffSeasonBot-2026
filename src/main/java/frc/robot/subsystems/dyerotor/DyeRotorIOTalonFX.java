
package frc.robot.subsystems.dyerotor;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.function.Supplier;

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
// import frc.robot.Constants;
// import frc.robot.util.CtreUtil;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;

public class DyeRotorIOTalonFX implements DyeRotorIO {
    public class DyeRotorConstants {
        public static final int kSpinMotorCANID = 52;
        public static final int kLeadIndexMotorCANID = 53;
        public static final int kFollowIndexMotorCANID = 54;
        public static final double kSpinSupplyCurrentLimit = 200;
        public static final double kSpinStatorCurrentLimit = 80;
        public static final double kSpinReduction = 2.5;
        public static final double kIndexSupplyCurrentLimit = 350;
        public static final double kIndexStatorCurrentLimit = 80;
        public static final double kIndexReduction = 36;
        public static final double kPSpin = 4.0;
        public static final double kVSpin = 900;
        public static final double MOI = 321.925;
    }
    protected final TalonFX m_spindexer =
        new TalonFX(DyeRotorConstants.kSpinMotorCANID, Constants.CanBuses.kLowerCANBus);
    protected final TalonFX m_indexerLead =
        new TalonFX(DyeRotorConstants.kLeadIndexMotorCANID, Constants.CanBuses.kLowerCANBus);
    protected final TalonFX m_indexerFollow =
        new TalonFX(DyeRotorConstants.kFollowIndexMotorCANID, Constants.CanBuses.kLowerCANBus);

    final VoltageOut m_indexerRequest = new VoltageOut(0);
    final VelocityVoltage m_spindexerRequest = new VelocityVoltage(0);
    
    public DyeRotorIOTalonFX() {
        configureMotors();
    }

    protected void configureMotors() {
        TalonFXConfiguration spinConfig = new TalonFXConfiguration();
        spinConfig.MotorOutput =
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);
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
                .withKP(DyeRotorConstants.kPSpin)
                .withKV(DyeRotorConstants.kVSpin);
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
        m_spindexer.getConfigurator().apply(spinConfig);
        m_indexerLead.getConfigurator().apply(indexConfig);
        m_indexerFollow.getConfigurator().apply(indexConfig);

        m_indexerFollow.setControl(new Follower(m_indexerLead.getDeviceID(), MotorAlignmentValue.Aligned));

    }

    @Override
    public void updateInputs(DyeRotorInputs inputs) {

        inputs.indexAppliedVolts = m_indexerLead.getMotorVoltage().refresh().getValueAsDouble();
        inputs.indexStatorCurrentAmps = m_indexerLead.getStatorCurrent().refresh().getValueAsDouble();
        inputs.indexSupplyCurrentAmps = m_indexerLead.getSupplyCurrent().refresh().getValueAsDouble();
    
        inputs.spinPositionRotations =
            motorRotationsToSpinMechanismRotations(m_spindexer.getPosition().refresh().getValueAsDouble());
        inputs.spinVelocityMetersPerSecond =
            motorRotationsToSpinMechanismRotations(m_spindexer.getVelocity().refresh().getValueAsDouble());
        inputs.spinAppliedVolts = m_spindexer.getMotorVoltage().refresh().getValueAsDouble();
        inputs.spinStatorCurrentAmps = m_spindexer.getStatorCurrent().refresh().getValueAsDouble();
        inputs.spinSupplyCurrentAmps = m_spindexer.getSupplyCurrent().refresh().getValueAsDouble();
    }

    @Override
    public void setSpinVelocity(double velocity) {
        m_spindexer.setControl(m_spindexerRequest.withVelocity(mechanismRotationsToSpinMotorRotations(velocity)));
    }

    @Override
    public void setIndexVoltage(double voltage) {
        m_indexerLead.setControl(m_indexerRequest.withOutput(voltage));
    }

    @Override
    public void stop() {
        m_indexerLead.stopMotor();
        m_indexerFollow.stopMotor();
        m_spindexer.stopMotor();
    }
    
    protected static double mechanismRotationsToSpinMotorRotations(double rotations) {
        return rotations * DyeRotorConstants.kSpinReduction;
    }
    protected static double motorRotationsToSpinMechanismRotations(double rotations) {
        return rotations / DyeRotorConstants.kSpinReduction;
    }
    // protected static double mechanismRotationsToIndexMotorRotations(double rotations) {
    //     return rotations * DyeRotorConstants.kIndexReduction;
    // }
}